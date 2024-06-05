# ECE780

## Install Dependencies
```
pip3 install -r requirements.txt
```

## Manual control
```
press `wasd` to control robot
press `rgb` to control led
press `+-` to open or close gripper
press `0` to reset gripper
press `enter` to reset arm
press `esc` to exit
```

## Turn on window permission for vscode running code via terminal
```
Reference: https://israynotarray.com/other/20200510/1067127387/

Open powershell with system administer
1. Get-ExecutionPolicy
Response: Restricted

2. Set-ExecutionPolicy RemoteSigned
Response: [Y] 是(Y)  [A] 全部皆是(A)  [N] 否(N)  [L] 全部皆否(L)  [S] 暫停(S)  [?] 說明 (預設值為 "N"):

3. Type Y

4. Get-ExecutionPolicy
Response: RemoteSigned
```

## Turn on virtualenv
```
# create
python3 -m virtualenv ece486_ece780t03_project_venv

# activate
ece486_ece780t03_project_venv\Scripts\activate

# deactive
deactivate
```

## Run project program
```
python .\main.py
```

## Axis of manipulator
```
Vertical axis is y (up is positive, down is negative)
Horizontal axis is x (forward is positive, backward is negative)
```