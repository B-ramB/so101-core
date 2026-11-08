"""USB camera detection."""

from pathlib import Path

from so101.prompts import ask_choice, header


def video_device_name(path: str) -> str:
    name_file = Path("/sys/class/video4linux") / Path(path).name / "name"
    try:
        return name_file.read_text().strip()
    except OSError:
        return "unknown"


def detect_camera() -> dict | None:
    header("Detecting cameras")
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

    cameras = OpenCVCamera.find_cameras()
    if not cameras:
        print("No camera found. Skipping camera setup.")
        return None

    options = []
    for cam in cameras:
        profile = cam.get("default_stream_profile", {})
        options.append(
            f"{cam['id']}  ({video_device_name(str(cam['id']))})  "
            f"{profile.get('width')}x{profile.get('height')} @ {profile.get('fps')} fps"
        )
    print("Found cameras (a built-in laptop webcam may also be listed):")
    index = ask_choice("Which camera should be used?", options, allow_skip=True)
    if index is None:
        return None

    cam = cameras[index]
    profile = cam.get("default_stream_profile", {})
    return {
        "type": "opencv",
        "index_or_path": str(cam["id"]),
        "name": video_device_name(str(cam["id"])),
        "width": int(profile.get("width") or 640),
        "height": int(profile.get("height") or 480),
        "fps": round(profile.get("fps") or 30),
    }
