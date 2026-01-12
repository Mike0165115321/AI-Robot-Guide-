/**
 * Test Script for TTS Chunking Logic (Min-200 Strategy)
 * Run with: node tests/test_tts_chunking.js
 */

console.log("🧪 Starting TTS Chunking Logic Test...");

// ---------------------------------------------------------
// The Logic to Test (Copied/Adapted from AvatarManager.js)
// ---------------------------------------------------------
function chunkText(text) {
    const cleanText = text;
    // Split by delimiters but keep them.
    const rawEvents = cleanText.split(/([ \n.!?]+)/).filter(s => s.length > 0);

    const TARGET_MIN_LENGTH = 200;
    let chunks = [];
    let currentBuffer = '';

    for (const event of rawEvents) {
        currentBuffer += event;

        // Check flush condition:
        // 1. Buffer size met target
        // 2. Current event is a delimiter (so we just finished a word/sentence)
        const isDelimiter = /^[ \n.!?]+$/.test(event);

        if (currentBuffer.length >= TARGET_MIN_LENGTH && isDelimiter) {
            // Flush
            chunks.push(currentBuffer);
            currentBuffer = '';
        }
    }

    // Push remaining
    if (currentBuffer.trim().length > 0) {
        chunks.push(currentBuffer);
    }
    return chunks;
}

// ---------------------------------------------------------
// Test Cases
// ---------------------------------------------------------

const tests = [
    {
        name: "Short Text (< 200)",
        input: "สวัสดีครับ วันนี้อากาศดีจังเลย",
        expectedCount: 1
    },
    {
        name: "Exact 200 Chars (No Delimiter at end)",
        // 200 'a's
        input: "a".repeat(200),
        expectedCount: 1
    },
    {
        name: "Long Text (300 chars) with spaces",
        // 150 chars + space + 150 chars
        input: "a".repeat(150) + " " + "b".repeat(150),
        // Should accumulate 150 -> wait -> add space -> still < 200? No 151.
        // Wait, logic: >= 200 AND delimiter.
        // 1. "a"*150 -> buff=150.
        // 2. " " -> buff=151. isDelimiter=True. Length < 200. No flush.
        // 3. "b"*150 -> buff=301. isDelimiter=False. No flush.
        // End -> Flush 301.
        // Result: 1 chunk.
        expectedCount: 1
    },
    {
        name: "Long Text (Multple Sentences) > 200",
        // Sentence 1 (~120 chars) . Sentence 2 (~120 chars) .
        input: "A".repeat(120) + ". " + "B".repeat(120) + ".",
        // 1. A*120 -> buff=120
        // 2. ". " -> buff=122. Delimiter. < 200. Keep.
        // 3. B*120 -> buff=242. Not Delimiter. Keep.
        // 4. "." -> buff=243. Delimiter. >= 200? Yes! Flush.
        // expecting 1 chunk?
        // Wait.
        // "A... . " (122) + "B..." (120) = 242.
        // Then "." (1). Total 243.
        // At step 4 (delimiter "."), buffer is 243. Flush.
        // So 1 chunk?
        // Let's try something that DEFINITELY chunks.
        // 250 chars + space + 50 chars.
        input: "A".repeat(250) + " " + "B".repeat(50),
        // 1. A*250 -> buff=250.
        // 2. " " -> buff=251. Delimiter=True. >=200=True. FLUSH.
        // 3. B*50 -> buff=50.
        // End -> Flush.
        // Total 2 chunks.
        expectedCount: 2
    }
];

let fullPass = true;

tests.forEach((t, i) => {
    console.log(`\nCase ${i + 1}: ${t.name}`);
    const result = chunkText(t.input);
    console.log(`   Input Length: ${t.input.length}`);
    console.log(`   Chunks: ${result.length}`);
    if (result.length === t.expectedCount) {
        console.log("   ✅ PASS");
    } else {
        console.log(`   ❌ FAIL (Expected ${t.expectedCount}, Got ${result.length})`);
        console.log("   Result:", result);
        fullPass = false;
    }
});

if (!fullPass) {
    console.error("\n❌ Some tests failed chunking verification.");
    process.exit(1);
} else {
    console.log("\n✅ All TTS Chunking logic tests passed.");
}
