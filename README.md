# Install

```
sudo cp robot.service /etc/systemd/system/
sudo systemctl daemon-reload 
sudo systemctl enable robot.service
```

# Deinstall

```
sudo systemctl disable robot-webcam.service
sudo rm -f /etc/systemd/system/robot.service
```

[BOM Sheets](https://docs.google.com/spreadsheets/d/1ZaKwszU7OfP2QDUP3J89k1ooDT7O5iXCyGr-KMcQ8w8/edit?usp=sharing)


# DISPLAY forward
run this on the actual display terminal `xhost +SI:localuser:john` and then use `DISPLAY=:0` flag to see the GUI