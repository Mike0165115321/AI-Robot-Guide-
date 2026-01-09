import json
import requests
import os

# Files
CREDS_FILE = "credentials.json"
TOKEN_FILE = "assistant_credentials.json"
PROJECT_ID = "still-toolbox-479616-e4"
MODEL_ID = "nan-robot-model-v1"
DEVICE_ID = "nan-robot-device-v1"

def get_access_token():
    try:
        with open(CREDS_FILE) as f:
            creds = json.load(f)["installed"]
        with open(TOKEN_FILE) as f:
            tokens = json.load(f)
        
        data = {
            "client_id": creds["client_id"],
            "client_secret": creds["client_secret"],
            "refresh_token": tokens["refresh_token"],
            "grant_type": "refresh_token"
        }
        resp = requests.post("https://oauth2.googleapis.com/token", data=data)
        if resp.status_code != 200:
             print(f"Failed to refresh token: {resp.text}")
             return None
        return resp.json()["access_token"]
    except Exception as e:
        print(f"Error loading tokens: {e}")
        return None

def register_model(access_token):
    url = f"https://embeddedassistant.googleapis.com/v1alpha2/projects/{PROJECT_ID}/deviceModels"
    headers = {"Authorization": f"Bearer {access_token}"}
    payload = {
        "project_id": PROJECT_ID,
        "deviceModelId": MODEL_ID,
        "manifest": {
            "manufacturer": "NanRobotDev",
            "productName": "NanRobot",
            "deviceDescription": "Nan Robot Assistant"
        },
        "deviceType": "action.devices.types.LIGHT",
        "traits": ["action.devices.traits.OnOff"]
    }
    print(f"DTO Register Model: {payload}")
    resp = requests.post(url, json=payload, headers=headers)
    print(f"Register Model Result: {resp.status_code} - {resp.text}")

def register_device(access_token):
    # Register device instance
    url = f"https://embeddedassistant.googleapis.com/v1alpha2/projects/{PROJECT_ID}/deviceModels/{MODEL_ID}/devices"
    headers = {"Authorization": f"Bearer {access_token}"}
    payload = {
        "id": DEVICE_ID,
        "nickname": "NanRobotUnit1",
        "clientType": "SDK_SERVICE"
    }
    print(f"DTO Register Device: {payload}")
    resp = requests.post(url, json=payload, headers=headers)
    print(f"Register Device Result: {resp.status_code} - {resp.text}")

def list_models(access_token):
    url = f"https://embeddedassistant.googleapis.com/v1alpha2/projects/{PROJECT_ID}/deviceModels"
    headers = {"Authorization": f"Bearer {access_token}"}
    resp = requests.get(url, headers=headers)
    print(f"List Models ({resp.status_code}):")
    if resp.status_code == 200:
        models = resp.json().get("deviceModels", [])
        print(f"Found {len(models)} models.")
        for m in models:
            print(f" - {m.get('deviceModelId')} ({m.get('projectId')})")
    else:
        print(resp.text)

token = get_access_token()
if token:
    print(f"Token acquired. Accessing Project: {PROJECT_ID}")
    # 1. Try to list models first (Test Auth/Project)
    list_models(token)
    
    # 2. Try to register
    # register_model(token) # Done
    register_device(token)
else:
    print("Could not get token.")
