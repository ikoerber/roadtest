"""Make Adafruit_BNO055::begin() return if the chip never reappears."""

from pathlib import Path

Import("env")


library_source = (
    Path(env.subst("$PROJECT_LIBDEPS_DIR"))
    / env.subst("$PIOENV")
    / "Adafruit BNO055"
    / "Adafruit_BNO055.cpp"
)

unbounded_wait = """  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    delay(10);
  }
"""

bounded_wait = """  const uint32_t resetStart = millis();
  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
    if (millis() - resetStart >= 1500) {
      return false;
    }
    delay(10);
  }
"""

if not library_source.exists():
    raise RuntimeError(
        "Adafruit BNO055 source not found; dependency resolution did not finish"
    )

source_text = library_source.read_text(encoding="utf-8")
if bounded_wait not in source_text:
    if unbounded_wait not in source_text:
        raise RuntimeError(
            "Unsupported Adafruit BNO055 version: reset loop was not recognized"
        )
    library_source.write_text(
        source_text.replace(unbounded_wait, bounded_wait, 1),
        encoding="utf-8",
    )
    print("Applied 1500 ms timeout to Adafruit BNO055 reset wait")
