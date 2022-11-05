def getCameraLists():
    from pymf import get_MF_devices

    device_list = get_MF_devices()

    for i, device_name in enumerate(device_list):
        print(f"opencv_index: {i}, device_name: {device_name}")