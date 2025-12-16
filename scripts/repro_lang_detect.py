from langdetect import detect_langs, DetectorFactory

# Set seed for reproducibility (same as in language_detector.py)
DetectorFactory.seed = 0

test_sentences = [
    "I want to go to something interesting.",
    "Hello world",
    "Sawasdee krub",
    "สวัสดีครับ",
    "Ni hao",
    "ไปเที่ยวไหนดี",
    "I want to go to Nan province"
]

print("--- Testing langdetect directly ---")
for text in test_sentences:
    try:
        results = detect_langs(text)
        print(f"'{text}' -> {results}")
    except Exception as e:
        print(f"'{text}' -> Error: {e}")
