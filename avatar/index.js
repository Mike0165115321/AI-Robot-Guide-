/**
 * # Avatar Dev - Entry Point
 * ไฟล์หลักสำหรับ import Avatar modules
 * 
 * วิธีใช้:
 * ```javascript
 * import NanAvatar from './avatar_dev/index.js';
 * NanAvatar.init();
 * NanAvatar.setMood('speaking');
 * ```
 */

import { AvatarController } from './core/AvatarController.js';

// สร้าง instance เดียว (Singleton)
const NanAvatar = new AvatarController();

// Export ทั้ง default และ named exports
export default NanAvatar;
export { AvatarController };

// Export modules สำหรับ advanced usage
export { MOOD_COLORS, TIMING } from './config/colors.js';
export { EyeTracking } from './core/EyeTracking.js';
export { BlinkController } from './core/BlinkController.js';
export { IdleManager } from './behaviors/IdleManager.js';

// Skins
export { BaseSkin } from './skins/BaseSkin.js';
export { NanRobotSkin } from './skins/NanRobot.js';
export { GoldenNanSkin } from './skins/GoldenNan.js';
export { NightNanSkin } from './skins/NightNan.js';
export { SakuraNanSkin } from './skins/SakuraNan.js';
export { CyberNanSkin } from './skins/CyberNan.js';
export { ForestNanSkin } from './skins/ForestNan.js';
export { OceanNanSkin } from './skins/OceanNan.js';
export { SunsetNanSkin } from './skins/SunsetNan.js';
export { FestivalNanSkin } from './skins/FestivalNan.js';

// Export moods
export { BaseMood } from './moods/BaseMood.js';
export { NormalMood } from './moods/NormalMood.js';
export { SpeakingMood } from './moods/SpeakingMood.js';
export { ThinkingMood } from './moods/ThinkingMood.js';
export { ListeningMood } from './moods/ListeningMood.js';
export { HappyMood } from './moods/HappyMood.js';
export { CuriousMood } from './moods/CuriousMood.js';
export { SleepyMood } from './moods/SleepyMood.js';
