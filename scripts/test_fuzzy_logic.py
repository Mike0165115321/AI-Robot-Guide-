import difflib

def test_fuzzy():
    query = "ดอสเสมอดาว"
    target = "ดอยเสมอดาว (อุทยานแห่งชาติศรีน่าน)"
    titles = [
        "แม่น้ำน่าน",
        "วัดภูมินทร์",
        target,
        "พระธาตุแช่แห้ง"
    ]
    
    print(f"--- Prefix Fuzzy Logic ---")
    
    # Simulate logic: Check similarity with title prefix of similar length
    best_match = None
    best_ratio = 0.0
    cutoff = 0.6
    
    processed_titles = []
    
    for title in titles:
        # Compare against prefix of title (with some slack e.g. +2 chars)
        compare_len = len(query) + 2
        title_prefix = title[:compare_len]
        
        ratio = difflib.SequenceMatcher(None, query, title_prefix).ratio()
        print(f"Checking '{title}' (Prefix: '{title_prefix}') -> Ratio: {ratio:.4f}")
        
        if ratio > best_ratio and ratio > cutoff:
            best_ratio = ratio
            best_match = title

    print(f"Best Match: {best_match} (Ratio: {best_ratio})")

if __name__ == "__main__":
    test_fuzzy()
