echo "=== 필수 패키지 설치 ==="
apt update
apt install -y lsb-release micro git curl build-essential cmake ca-certificates python3-pip python3-catkin-tools

echo "=== 인증서 업데이트 ==="
update-ca-certificates


echo "=== 환경 변수 설정 ==="
if ! grep -q "alias eb='micro ~/.bashrc'" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "alias eb='micro ~/.bashrc'" >> ~/.bashrc
    echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
    echo "alias cm='catkin_make'" >> ~/.bashrc
    echo "alias sd='source devel/setup.bash'" >> ~/.bashrc
    echo "alias cw='cd ~/share/catkin_ws'" >> ~/.bashrc
    echo "alias rscam='roslaunch realsense2_camera rs_camera.launch align_depth:=true'" >> ~/.bashrc
    echo "export DISPLAY=:0" >> ~/.bashrc
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
fi