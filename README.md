# Install

```
sudo cp robot.service /etc/systemd/system/
sudo systemctl daemon-reload 
sudo systemctl enable robot.service
```

# Deinstall

sudo systemctl disable robot-webcam.service
sudo rm -f /etc/systemd/system/robot.service