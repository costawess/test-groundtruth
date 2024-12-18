from ota import OTAUpdater
from WIFI_CONFIG import SSID, PASSWORD

firmware_url = "https://github.com/costawess/test-groundtruth/"

ota_updater = OTAUpdater(SSID, PASSWORD, firmware_url, "robot.py")

ota_updater.download_and_install_update_if_available()
