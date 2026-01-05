
class SpeechService {
    constructor() {
        this.mediaRecorder = null;
        this.audioChunks = [];
        this.stream = null;
    }

    async startRecording() {
        try {
            this.stream = await navigator.mediaDevices.getUserMedia({ audio: true });
            this.mediaRecorder = new MediaRecorder(this.stream);
            this.audioChunks = [];

            this.mediaRecorder.ondataavailable = (event) => {
                this.audioChunks.push(event.data);
            };

            this.mediaRecorder.start();
            return true;
        } catch (err) {
            console.error('Microphone Error:', err);
            return false;
        }
    }

    stopRecording() {
        return new Promise((resolve) => {
            if (!this.mediaRecorder) return resolve(null);

            this.mediaRecorder.onstop = () => {
                const audioBlob = new Blob(this.audioChunks, { type: 'audio/wav' }); // or 'audio/webm' depending on browser
                this._stopStream();
                resolve(audioBlob);
            };

            this.mediaRecorder.stop();
        });
    }

    _stopStream() {
        if (this.stream) {
            this.stream.getTracks().forEach(track => track.stop());
            this.stream = null;
        }
    }

    playAudio(url) {
        const audio = new Audio(url);
        return audio.play();
    }
}

export const speechService = new SpeechService();
export default speechService;
