Import("env")

from pathlib import Path

libdeps_dir = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV")
source_path = libdeps_dir / "DShotRMT" / "DShotRMT.cpp"

if source_path.exists():
    source = source_path.read_text(encoding="utf-8")
    source = source.replace(
        "static_cast<uint8_t>(RMT_CHANNEL_MAX - static_cast<uint8_t>(rmtChannel))",
        "1",
    )
    source = source.replace("RMT_CHANNEL_MAX - channel", "1")
    source = source.replace("RMT_CHANNEL_MAX - 1", "RMT_CHANNEL_MAX - 1")
    source = source.replace(
        "if (throttle_value < DSHOT_THROTTLE_MIN)",
        "if (throttle_value != 0 && throttle_value < DSHOT_THROTTLE_MIN)",
    )
    source_path.write_text(source, encoding="utf-8")
