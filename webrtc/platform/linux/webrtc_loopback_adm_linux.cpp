// This file is part of Desktop App Toolkit,
// a set of libraries for developing nice desktop applications.
//
// For license and copyright information please follow this link:
// https://github.com/desktop-app/legal/blob/master/LEGAL
//
#include "webrtc/platform/linux/webrtc_loopback_adm_linux.h"

#include <al.h>
#include <alc.h>

#include <rtc_base/logging.h>

#include <chrono>
#include <cstring>
#include <vector>

namespace Webrtc::details {
namespace {

constexpr auto kSampleRate = 48000;
constexpr auto kChannels = 2;
constexpr auto kBufferSizeMs = 10;
constexpr auto kFramesPerChunk = (kSampleRate * kBufferSizeMs) / 1000; // 480
constexpr auto kCaptureBufferFrames = kFramesPerChunk * 4; // 40ms ring buffer
constexpr ALenum kCaptureFormat = AL_FORMAT_STEREO16;

} // namespace

AudioDeviceLoopbackLinux::AudioDeviceLoopbackLinux(
	webrtc::TaskQueueFactory *taskQueueFactory)
: _audioDeviceBuffer(taskQueueFactory) {
}

AudioDeviceLoopbackLinux::~AudioDeviceLoopbackLinux() {
	Terminate();
}

// static
bool AudioDeviceLoopbackLinux::IsSupported() {
	return !FindMonitorDevice().empty();
}

// static
std::string AudioDeviceLoopbackLinux::FindMonitorDevice() {
	// ALC_CAPTURE_DEVICE_SPECIFIER returns a double-null-terminated list
	// of device name strings when queried against nullptr.
	const auto *deviceList = alcGetString(nullptr, ALC_CAPTURE_DEVICE_SPECIFIER);
	if (!deviceList) {
		return {};
	}

	while (*deviceList) {
		const auto name = std::string(deviceList);
		// PulseAudio monitor sources contain "monitor" in their OpenAL name.
		if (name.find("monitor") != std::string::npos
				|| name.find("Monitor") != std::string::npos) {
			return name;
		}
		deviceList += name.size() + 1;
	}
	return {};
}

int32_t AudioDeviceLoopbackLinux::ActiveAudioLayer(
		AudioLayer *audioLayer) const {
	*audioLayer = AudioDeviceModule::kPlatformDefaultAudio;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::RegisterAudioCallback(
		webrtc::AudioTransport *audioCallback) {
	return _audioDeviceBuffer.RegisterAudioCallback(audioCallback);
}

int32_t AudioDeviceLoopbackLinux::Init() {
	if (_initialized) {
		return 0;
	}
	_captureDeviceId = FindMonitorDevice();
	if (_captureDeviceId.empty()) {
		RTC_LOG(LS_ERROR)
			<< "AudioDeviceLoopbackLinux::Init: "
			<< "no OpenAL monitor capture device found.";
		return -1;
	}
	_audioDeviceBuffer.SetRecordingSampleRate(kSampleRate);
	_audioDeviceBuffer.SetRecordingChannels(kChannels);
	_initialized = true;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::Terminate() {
	if (!_initialized) {
		return 0;
	}
	StopRecording();
	_initialized = false;
	_recordingInitialized = false;
	_microphoneInitialized = false;
	return 0;
}

bool AudioDeviceLoopbackLinux::Initialized() const {
	return _initialized;
}

int16_t AudioDeviceLoopbackLinux::PlayoutDevices() {
	return 0;
}

int16_t AudioDeviceLoopbackLinux::RecordingDevices() {
	return 1;
}

int32_t AudioDeviceLoopbackLinux::PlayoutDeviceName(
		uint16_t index,
		char name[webrtc::kAdmMaxDeviceNameSize],
		char guid[webrtc::kAdmMaxGuidSize]) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::RecordingDeviceName(
		uint16_t index,
		char name[webrtc::kAdmMaxDeviceNameSize],
		char guid[webrtc::kAdmMaxGuidSize]) {
	if (index != 0) {
		return -1;
	}
	const auto length = std::min(
		_captureDeviceId.size(),
		std::size_t(webrtc::kAdmMaxDeviceNameSize - 1));
	std::memcpy(name, _captureDeviceId.data(), length);
	name[length] = '\0';
	if (guid) {
		guid[0] = '\0';
	}
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetPlayoutDevice(uint16_t index) {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetPlayoutDevice(
		WindowsDeviceType device) {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetRecordingDevice(uint16_t index) {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetRecordingDevice(
		WindowsDeviceType device) {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::PlayoutIsAvailable(bool *available) {
	*available = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::InitPlayout() {
	return 0;
}

bool AudioDeviceLoopbackLinux::PlayoutIsInitialized() const {
	return false;
}

int32_t AudioDeviceLoopbackLinux::RecordingIsAvailable(bool *available) {
	*available = _initialized;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::InitRecording() {
	if (!_initialized) {
		return -1;
	}
	if (_recordingInitialized) {
		return 0;
	}
	_recordingInitialized = true;
	return 0;
}

bool AudioDeviceLoopbackLinux::RecordingIsInitialized() const {
	return _recordingInitialized;
}

int32_t AudioDeviceLoopbackLinux::StartPlayout() {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::StopPlayout() {
	return 0;
}

bool AudioDeviceLoopbackLinux::Playing() const {
	return false;
}

int32_t AudioDeviceLoopbackLinux::StartRecording() {
	if (!_recordingInitialized) {
		return -1;
	}
	if (_recording) {
		return 0;
	}
	_shouldStop.store(false);
	_audioDeviceBuffer.StartRecording();
	_captureThread = std::thread(&AudioDeviceLoopbackLinux::captureLoop, this);
	_recording = true;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::StopRecording() {
	if (!_recording) {
		return 0;
	}
	_shouldStop.store(true);
	if (_captureThread.joinable()) {
		_captureThread.join();
	}
	_audioDeviceBuffer.StopRecording();
	_recording = false;
	return 0;
}

bool AudioDeviceLoopbackLinux::Recording() const {
	return _recording;
}

int32_t AudioDeviceLoopbackLinux::InitSpeaker() {
	return 0;
}

bool AudioDeviceLoopbackLinux::SpeakerIsInitialized() const {
	return false;
}

int32_t AudioDeviceLoopbackLinux::InitMicrophone() {
	if (!_initialized) {
		return -1;
	}
	_microphoneInitialized = true;
	return 0;
}

bool AudioDeviceLoopbackLinux::MicrophoneIsInitialized() const {
	return _microphoneInitialized;
}

int32_t AudioDeviceLoopbackLinux::SpeakerVolumeIsAvailable(bool *available) {
	*available = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetSpeakerVolume(uint32_t volume) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::SpeakerVolume(uint32_t *volume) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MaxSpeakerVolume(
		uint32_t *maxVolume) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MinSpeakerVolume(
		uint32_t *minVolume) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MicrophoneVolumeIsAvailable(
		bool *available) {
	*available = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetMicrophoneVolume(uint32_t volume) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MicrophoneVolume(
		uint32_t *volume) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MaxMicrophoneVolume(
		uint32_t *maxVolume) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MinMicrophoneVolume(
		uint32_t *minVolume) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MicrophoneMuteIsAvailable(
		bool *available) {
	*available = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetMicrophoneMute(bool enable) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::MicrophoneMute(bool *enabled) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::SpeakerMuteIsAvailable(bool *available) {
	*available = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetSpeakerMute(bool enable) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::SpeakerMute(bool *enabled) const {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::StereoPlayoutIsAvailable(
		bool *available) const {
	*available = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetStereoPlayout(bool enable) {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::StereoPlayout(bool *enabled) const {
	*enabled = false;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::StereoRecordingIsAvailable(
		bool *available) const {
	*available = true;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::SetStereoRecording(bool enable) {
	return 0;
}

int32_t AudioDeviceLoopbackLinux::StereoRecording(bool *enabled) const {
	*enabled = true;
	return 0;
}

int32_t AudioDeviceLoopbackLinux::PlayoutDelay(uint16_t *delayMS) const {
	*delayMS = 0;
	return 0;
}

bool AudioDeviceLoopbackLinux::BuiltInAECIsAvailable() const {
	return false;
}

bool AudioDeviceLoopbackLinux::BuiltInAGCIsAvailable() const {
	return false;
}

bool AudioDeviceLoopbackLinux::BuiltInNSIsAvailable() const {
	return false;
}

int32_t AudioDeviceLoopbackLinux::EnableBuiltInAEC(bool enable) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::EnableBuiltInAGC(bool enable) {
	return -1;
}

int32_t AudioDeviceLoopbackLinux::EnableBuiltInNS(bool enable) {
	return -1;
}

void AudioDeviceLoopbackLinux::captureLoop() {
	ALCdevice *device = alcCaptureOpenDevice(
		_captureDeviceId.c_str(),
		kSampleRate,
		kCaptureFormat,
		kCaptureBufferFrames);
	if (!device) {
		RTC_LOG(LS_ERROR)
			<< "AudioDeviceLoopbackLinux::captureLoop: "
			<< "alcCaptureOpenDevice failed for '"
			<< _captureDeviceId
			<< "'";
		return;
	}

	alcCaptureStart(device);
	if (alcGetError(device) != ALC_NO_ERROR) {
		RTC_LOG(LS_ERROR)
			<< "AudioDeviceLoopbackLinux::captureLoop: "
			<< "alcCaptureStart failed.";
		alcCaptureCloseDevice(device);
		return;
	}

	// Interleaved stereo int16 samples: kFramesPerChunk frames * kChannels
	auto buffer = std::vector<int16_t>(kFramesPerChunk * kChannels);

	while (!_shouldStop.load(std::memory_order_acquire)) {
		auto available = ALCint(0);
		alcGetIntegerv(device, ALC_CAPTURE_SAMPLES, 1, &available);

		if (alcGetError(device) != ALC_NO_ERROR) {
			RTC_LOG(LS_ERROR)
				<< "AudioDeviceLoopbackLinux::captureLoop: "
				<< "alcGetIntegerv(ALC_CAPTURE_SAMPLES) failed, stopping.";
			break;
		}

		if (available < ALCint(kFramesPerChunk)) {
			// Not enough data for a full 10 ms chunk yet; yield briefly.
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

		alcCaptureSamples(
			device,
			reinterpret_cast<ALCvoid *>(buffer.data()),
			kFramesPerChunk);
		if (alcGetError(device) != ALC_NO_ERROR) {
			RTC_LOG(LS_ERROR)
				<< "AudioDeviceLoopbackLinux::captureLoop: "
				<< "alcCaptureSamples failed, stopping.";
			break;
		}

		// Deliver one 10 ms frame to the WebRTC pipeline.
		// SetRecordedBuffer expects the sample count per channel.
		_audioDeviceBuffer.SetRecordedBuffer(buffer.data(), kFramesPerChunk);
		_audioDeviceBuffer.SetVQEData(0, 0);
		_audioDeviceBuffer.DeliverRecordedData();
	}

	alcCaptureStop(device);
	alcCaptureCloseDevice(device);
}

} // namespace Webrtc::details