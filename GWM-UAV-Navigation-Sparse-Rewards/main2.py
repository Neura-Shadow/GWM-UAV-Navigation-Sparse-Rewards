import cosysairsim as airsim # 使用 cosys 專屬導入方式
import os
import subprocess

def get_wsl_host_ip():
    # 優先使用 WSL2 預設閘道 (Windows Host)
    try:
        output = subprocess.check_output(["ip", "route", "show", "default"], text=True)
        for line in output.splitlines():
            parts = line.split()
            if "via" in parts:
                return parts[parts.index("via") + 1]
    except Exception:
        pass
    # 回退到 resolv.conf 的 nameserver
    try:
        with open("/etc/resolv.conf", "r") as f:
            for line in f:
                if "nameserver" in line:
                    return line.split()[1]
    except Exception:
        pass
    return "127.0.0.1"

host_ip = os.environ.get("AIRSIM_HOST", "") or get_wsl_host_ip()
api_port = 41451
vehicle_name = os.environ.get("AIRSIM_VEHICLE", "SimpleFlight")
client = airsim.MultirotorClient(ip=host_ip, port=api_port)
try:
    print(f"嘗試連線到 Cosys-AirSim: {host_ip}:{api_port}")
    client.confirmConnection()
    print("Cosys-AirSim 連線成功！")

    # 顯示 UE 實際載入的 settings 摘要，快速確認是否有 Lidar 設定
    settings_text = None
    if hasattr(client, "getSettingsString"):
        try:
            settings_text = client.getSettingsString()
            has_sensors_key = '"Sensors"' in settings_text
            has_lidar_sensor = "LidarSensor1" in settings_text
            has_legacy_lidar = '"Lidar"' in settings_text
            has_lidar1 = "Lidar1" in settings_text
            print(
                "Settings 檢查: "
                f"Sensors鍵={has_sensors_key}, LidarSensor1={has_lidar_sensor}, "
                f"Lidar鍵={has_legacy_lidar}, Lidar1={has_lidar1}"
            )
            if not (has_sensors_key or has_legacy_lidar):
                print("UE 目前載入的 settings 內容如下（節錄）:")
                print(settings_text[:400].strip())
                print(
                    "\n偵測到 UE 未載入你的 Lidar 設定。"
                    "請用啟動參數 -settings 指向正確檔案，例如：\n"
                    "UnrealEditor.exe \"D:\\Program\\Unreal projects\\UAVSIM\\UAVSIM.uproject\" "
                    "-settings=\"C:\\Users\\zongx\\OneDrive\\文件\\AirSim\\settings.json\""
                )
        except Exception as settings_err:
            print(f"無法取得 UE settings 內容: {settings_err}")
    else:
        print("此 cosysairsim 版本不支援 getSettingsString()")
    
    # 列出 UE 端目前可見車輛（若 API 支援）
    available_vehicles = []
    try:
        available_vehicles = client.listVehicles()
        print(f"UE 回報的車輛清單: {available_vehicles}")
    except Exception:
        pass

    # 優先使用環境變數指定車輛，其次用 UE 回報第一台
    if os.environ.get("AIRSIM_VEHICLE", ""):
        selected_vehicle = vehicle_name
    elif available_vehicles:
        selected_vehicle = available_vehicles[0]
    else:
        selected_vehicle = vehicle_name

    print(f"使用車輛名稱: {selected_vehicle}")
    try:
        client.enableApiControl(True, selected_vehicle)
        client.armDisarm(True, selected_vehicle)
    except Exception as vehicle_err:
        if available_vehicles and selected_vehicle != available_vehicles[0]:
            fallback_vehicle = available_vehicles[0]
            print(
                f"指定車輛 '{selected_vehicle}' 不可用，改用 UE 車輛 '{fallback_vehicle}'。"
            )
            selected_vehicle = fallback_vehicle
            client.enableApiControl(True, selected_vehicle)
            client.armDisarm(True, selected_vehicle)
        else:
            raise vehicle_err

    # 獲取 Lidar 數據 (需要在 settings.json 內配置 Lidar)
    requested_lidar = os.environ.get("AIRSIM_LIDAR", "Lidar1")
    lidar_candidates = [requested_lidar, "LidarSensor1", "Lidar1", "Lidar"]
    lidar_candidates = list(dict.fromkeys(lidar_candidates))

    success = False
    last_error = None
    for lidar_name in lidar_candidates:
        try:
            lidar_data = client.getLidarData(lidar_name, selected_vehicle)
            print(
                f"Vehicle: {selected_vehicle}, Lidar: {lidar_name}, "
                f"掃描到點雲數量: {len(lidar_data.point_cloud)}"
            )
            success = True
            break
        except Exception as lidar_err:
            last_error = lidar_err

    if not success:
        try:
            lidar_data = client.getLidarData("", selected_vehicle)
            print(
                f"Vehicle: {selected_vehicle}, Lidar: <default>, "
                f"掃描到點雲數量: {len(lidar_data.point_cloud)}"
            )
            success = True
        except Exception as lidar_err:
            last_error = lidar_err

    if not success:
        print(
            "仍無法讀取 Lidar，通常是 Cosys 設定格式與版本不匹配（例如 SettingsVersion 2.0 仍使用舊版 Lidar 區塊）。\n"
            f"嘗試的車輛名稱: {selected_vehicle}\n"
            f"嘗試的 Lidar 名稱: {lidar_candidates + ['<default>']}\n"
            f"最後錯誤: {last_error}"
        )
    
except Exception as e:
    print(
        "請確認 Cosys-AirSim UE 已啟動且 Settings.json 的 ApiServerPort 正確。\n"
        f"連線目標: {host_ip}:{api_port}\n"
        f"錯誤: {e}"
    )