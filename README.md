# Install

```
sudo cp robot.service /etc/systemd/system/
sudo systemctl daemon-reload 
sudo systemctl enable robot.service
```

# Deinstall

```
sudo systemctl disable robot.service
sudo rm -f /etc/systemd/system/robot.service
```


# Create .urdf file 

```
xacrodoc urdf/robot/urdf/robot.xacro > urdf/robot/urdf/robot.urdf
```