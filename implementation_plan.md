# Implementation Plan - Standardizing Frontend Theme

The goal is to apply the new "Futuristic AI" design system (`main.css`) to all pages (`chat`, `admin`, `travel_mode`) to ensure a consistent user experience.

## Proposed Changes

### 1. Chat Page
#### [MODIFY] [chat.html](file:///home/mikedev/AI%20Robot%20Guide%20%E0%B8%88%E0%B8%B1%E0%B8%87%E0%B8%AB%E0%B8%A7%E0%B8%B1%E0%B8%94%E0%B8%99%E0%B9%88%E0%B8%B2%E0%B8%99/frontend/chat.html)
- Add `<link rel="stylesheet" href="assets/styles/main.css">`.
- Update structure to use `main.css` classes where applicable.

#### [MODIFY] [chat.css](file:///home/mikedev/AI%20Robot%20Guide%20%E0%B8%88%E0%B8%B1%E0%B8%87%E0%B8%AB%E0%B8%A7%E0%B8%B1%E0%B8%94%E0%B8%99%E0%B9%88%E0%B8%B2%E0%B8%99/frontend/assets/styles/chat.css)
- Remove conflicting root variables (inherit from `main.css`).
- Update Sidebar to use Glassmorphism (`var(--glass)`).
- Update Chat Bubbles to match the theme.
- Update Input Area to be floating and glass-like.

### 2. Admin Page
#### [MODIFY] [admin.html](file:///home/mikedev/AI%20Robot%20Guide%20%E0%B8%88%E0%B8%B1%E0%B8%87%E0%B8%AB%E0%B8%A7%E0%B8%B1%E0%B8%94%E0%B8%99%E0%B9%88%E0%B8%B2%E0%B8%99/frontend/admin.html)
- Add `<link rel="stylesheet" href="assets/styles/main.css">`.

#### [MODIFY] [admin.css](file:///home/mikedev/AI%20Robot%20Guide%20%E0%B8%88%E0%B8%B1%E0%B8%87%E0%B8%AB%E0%B8%A7%E0%B8%B1%E0%B8%94%E0%B8%99%E0%B9%88%E0%B8%B2%E0%B8%99/frontend/assets/styles/admin.css)
- Update Table styles to be transparent/glass.
- Update Forms to use neon borders on focus.
- Update Buttons to use `main.css` button styles.

### 3. Travel Mode Page
#### [MODIFY] [travel_mode.html](file:///home/mikedev/AI%20Robot%20Guide%20%E0%B8%88%E0%B8%B1%E0%B8%87%E0%B8%AB%E0%B8%A7%E0%B8%B1%E0%B8%94%E0%B8%99%E0%B9%88%E0%B8%B2%E0%B8%99/frontend/travel_mode.html)
- Add `<link rel="stylesheet" href="assets/styles/main.css">`.

#### [MODIFY] [travel_mode.css](file:///home/mikedev/AI%20Robot%20Guide%20%E0%B8%88%E0%B8%B1%E0%B8%87%E0%B8%AB%E0%B8%A7%E0%B8%B1%E0%B8%94%E0%B8%99%E0%B9%88%E0%B8%B2%E0%B8%99/frontend/assets/styles/travel_mode.css)
- Update Map container to blend with the dark theme.
- Update Overlay controls to use Glassmorphism.

## Verification Plan
- Manually check each page to ensure the theme is applied correctly.
- Verify that functionality (Chat, Admin forms, Map) still works after style changes.
