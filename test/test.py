import xapi.api as x5
# 获取 IP 地址
ip_list = x5.get_ip_list()
print("IP 数量:", len(ip_list))
for i in range(0, len(ip_list)):
    print(f"目标[{i}] 网口：{ip_list[i]['net_no']}，IP：{ip_list[i]['ip']}")

# 机器人 IP 地址
ip = "168.168.40.20"  # 根据实际修改
# ip = "192.168.56.60"  # 根据实际修改
# 创建连接
handle = x5.connect(ip)
print(f"机器人句柄号：{handle}")