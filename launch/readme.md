使用*run_helmet_mid.launch*即可，对于仿真数据使用marsim.yaml，对于真实数据使用mid360.yaml
重命名当前文件夹下的pcd文件（从0开始）：
ls -v *.pcd | awk -v start=0 '{printf "mv -n \"%s\" \"%d.pcd\"\n", $0, NR+start-1}' | bash