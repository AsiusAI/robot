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

# Ideas

1. export urdf with correct weight

2. add linear_vel to obs space

3. normalize obs space

4. Continuous Torque Control 

5. Penalty for Angular Velocity

6. 3.1 Algorithmic Implementation and Best Practices

7. Weight Initialization

8. Curriculum Learning (make it first learn just balancing then staying in place)

