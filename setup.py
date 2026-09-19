from setuptools import setup

# Runtime dependencies are installed through rosdep/apt on Noetic (Python 3.8).
setup(name="zed_mid360_calibration", version="0.1.0",
      packages=["zed_mid360_calibration"], python_requires=">=3.8",
      entry_points={"console_scripts": [
          "zed-mid360-calibrate=zed_mid360_calibration.cli:main"]})
