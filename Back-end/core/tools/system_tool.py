# /core/tools/system_tool.py (Upgraded from app_launcher.py)
import subprocess
import logging
import platform
import webbrowser

class SystemTool:
    def __init__(self):
        self.current_os = platform.system().lower()
        self.is_wsl = 'microsoft' in platform.uname().release.lower()
        env_string = f"WSL ({self.current_os})" if self.is_wsl else self.current_os
        logging.info(f"⚙️  SystemTool initialized for environment: {env_string}")

        self.app_alias_map = {
            'เครื่องคิดเลข': 'calculator', 'คิดเลข': 'calculator',
            'โน้ตแพด': 'text_editor', 'จดบันทึก': 'text_editor', 'เท็กซ์เอดิเตอร์': 'text_editor',
            'ไฟล์': 'file_explorer', 'เปิดไฟล์': 'file_explorer',
            'เทอร์มินัล': 'terminal',
            'โครม': 'browser', 'เบราว์เซอร์': 'browser'
        }

        self.app_command_map = {
            'calculator': 'gnome-calculator',
            'text_editor': 'gedit',
            'file_explorer': 'nautilus .',
            'terminal': 'gnome-terminal',
            'browser': 'google-chrome-stable'
        }
        
        if self.is_wsl:
            self.app_command_map['file_explorer'] = 'explorer.exe .'
            self.app_command_map['browser'] = 'wslview'

        self.website_map = {
            'youtube': 'https://www.youtube.com',
            'google': 'https://www.google.com',
            'facebook': 'https://www.facebook.com',
        }

    def get_known_app_aliases(self) -> list[str]:
        """Returns all known ways to call an app."""
        return list(self.app_alias_map.keys())

    def launch(self, entity_name: str) -> str:
        normalized_entity = entity_name.lower().strip()
        
        if normalized_entity in self.website_map:
            url = self.website_map[normalized_entity]
            try:
                if self.is_wsl:
                    subprocess.Popen(['wslview', url])
                else:
                    webbrowser.open_new_tab(url)
                return f"กำลังเปิดเว็บไซต์ {normalized_entity.capitalize()} ให้ค่ะ"
            except Exception as e:
                logging.error(f"❌ [SystemTool] Failed to open website '{url}': {e}")
                return f"ขออภัยค่ะ เกิดข้อผิดพลาดขณะพยายามเปิดเว็บไซต์ {normalized_entity.capitalize()}"

        app_key = self.app_alias_map.get(normalized_entity, normalized_entity)
        command = self.app_command_map.get(app_key)

        if not command:
            return f"ขออภัยค่ะ หนูไม่รู้จักวิธีเปิด '{entity_name}' บนระบบนี้"

        try:
            subprocess.Popen(command, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            logging.info(f"✅ [SystemTool] Launched command: '{command}'")
            return f"กำลังเปิด {entity_name} ให้แล้วค่ะ"
        except FileNotFoundError:
            return f"ขออภัยค่ะ ดูเหมือนว่าโปรแกรมสำหรับ '{command}' จะยังไม่ได้ติดตั้ง"
        except Exception as e:
            logging.error(f"❌ [SystemTool] Failed to launch '{command}': {e}")
            return f"ขออภัยค่ะ เกิดข้อผิดพลาดขณะพยายามเปิด {entity_name}"

system_tool_instance = SystemTool()