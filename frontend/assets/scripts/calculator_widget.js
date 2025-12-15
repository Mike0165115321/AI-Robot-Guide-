// /frontend/assets/scripts/calculator_widget.js
// Scientific Calculator Widget - Separated for maintainability

class CalculatorWidget {
    /**
     * Create the Calculator Widget (Scientific Calculator)
     * @returns {HTMLElement} Container element with the calculator
     */
    static create() {
        const container = document.createElement('div');
        container.className = 'fab-widget-container';

        container.innerHTML = `
            <div class="bubble system-bubble" style="max-width: 380px; padding: 0; overflow: hidden;">
                <div style="background: linear-gradient(135deg, #1e293b, #0f172a); padding: 15px;">
                    <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px;">
                        <h3 style="margin: 0; font-size: 1rem; color: #10b981;">🔢 เครื่องคิดเลขคณิตศาสตร์</h3>
                        <button class="close-widget-btn-icon" style="background: none; border: none; color: #aaa; font-size: 1.2rem; cursor: pointer;">&times;</button>
                    </div>
                    
                    <!-- Display -->
                    <div class="calc-display" style="
                        background: #0f172a;
                        border: 1px solid rgba(16, 185, 129, 0.3);
                        border-radius: 8px;
                        padding: 12px;
                        margin-bottom: 12px;
                        text-align: right;
                        font-family: 'Consolas', monospace;
                    ">
                        <div class="calc-expression" style="font-size: 0.8rem; color: #64748b; min-height: 18px; word-break: break-all;"></div>
                        <div class="calc-result" style="font-size: 1.8rem; color: #f1f5f9; font-weight: bold; word-break: break-all;">0</div>
                    </div>
                    
                    <!-- Scientific Buttons Row 1 -->
                    <div class="calc-buttons" style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 6px;">
                        <button class="calc-btn sci" data-value="sin" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">sin</button>
                        <button class="calc-btn sci" data-value="cos" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">cos</button>
                        <button class="calc-btn sci" data-value="tan" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">tan</button>
                        <button class="calc-btn sci" data-value="log" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">log</button>
                        <button class="calc-btn sci" data-value="ln" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">ln</button>
                    </div>
                    
                    <!-- Scientific Buttons Row 2 -->
                    <div class="calc-buttons" style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 6px;">
                        <button class="calc-btn sci" data-value="sqrt" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa; font-size: 0.9rem;">√</button>
                        <button class="calc-btn sci" data-value="sq" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa; font-size: 0.8rem;">x²</button>
                        <button class="calc-btn sci" data-value="pow" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa; font-size: 0.8rem;">xʸ</button>
                        <button class="calc-btn sci" data-value="pi" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa; font-size: 0.9rem;">π</button>
                        <button class="calc-btn sci" data-value="e" style="background: rgba(168, 85, 247, 0.2); color: #a78bfa; font-size: 0.9rem;">e</button>
                    </div>
                    
                    <!-- Scientific Buttons Row 3 -->
                    <div class="calc-buttons" style="display: grid; grid-template-columns: repeat(5, 1fr); gap: 6px; margin-bottom: 6px;">
                        <button class="calc-btn sci" data-value="(" style="background: rgba(148, 163, 184, 0.15); color: #94a3b8;">(</button>
                        <button class="calc-btn sci" data-value=")" style="background: rgba(148, 163, 184, 0.15); color: #94a3b8;">)</button>
                        <button class="calc-btn sci" data-value="inv" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">1/x</button>
                        <button class="calc-btn sci" data-value="abs" style="background: rgba(96, 165, 250, 0.2); color: #60a5fa; font-size: 0.8rem;">|x|</button>
                        <button class="calc-btn func" data-value="backspace" style="background: rgba(239, 68, 68, 0.2); color: #f87171; font-size: 0.9rem;">⌫</button>
                    </div>
                    
                    <!-- Main Calculator Buttons -->
                    <div class="calc-buttons" style="display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px;">
                        <button class="calc-btn func" data-value="C" style="background: rgba(239, 68, 68, 0.3); color: #f87171;">C</button>
                        <button class="calc-btn func" data-value="±" style="background: rgba(148, 163, 184, 0.2); color: #94a3b8;">±</button>
                        <button class="calc-btn func" data-value="%" style="background: rgba(148, 163, 184, 0.2); color: #94a3b8;">%</button>
                        <button class="calc-btn op" data-value="/" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">÷</button>
                        
                        <button class="calc-btn num" data-value="7">7</button>
                        <button class="calc-btn num" data-value="8">8</button>
                        <button class="calc-btn num" data-value="9">9</button>
                        <button class="calc-btn op" data-value="*" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">×</button>
                        
                        <button class="calc-btn num" data-value="4">4</button>
                        <button class="calc-btn num" data-value="5">5</button>
                        <button class="calc-btn num" data-value="6">6</button>
                        <button class="calc-btn op" data-value="-" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">−</button>
                        
                        <button class="calc-btn num" data-value="1">1</button>
                        <button class="calc-btn num" data-value="2">2</button>
                        <button class="calc-btn num" data-value="3">3</button>
                        <button class="calc-btn op" data-value="+" style="background: rgba(251, 191, 36, 0.3); color: #fbbf24;">+</button>
                        
                        <button class="calc-btn num" data-value="0" style="grid-column: span 2;">0</button>
                        <button class="calc-btn num" data-value=".">.</button>
                        <button class="calc-btn eq" data-value="=" style="background: linear-gradient(135deg, #10b981, #059669); color: white;">=</button>
                    </div>
                </div>
            </div>
        `;

        // Apply button styles
        const style = document.createElement('style');
        style.textContent = `
            .calc-btn {
                padding: 12px 8px;
                border: none;
                border-radius: 8px;
                font-size: 1rem;
                font-weight: 600;
                cursor: pointer;
                transition: all 0.15s;
                font-family: 'Kanit', sans-serif;
            }
            .calc-btn.num {
                background: rgba(255, 255, 255, 0.1);
                color: #f1f5f9;
            }
            .calc-btn.sci {
                padding: 10px 6px;
            }
            .calc-btn:hover {
                transform: scale(1.05);
                filter: brightness(1.2);
            }
            .calc-btn:active {
                transform: scale(0.95);
            }
        `;
        container.appendChild(style);

        // Calculator State
        let currentValue = '0';
        let previousValue = '';
        let operator = '';
        let shouldResetDisplay = false;
        let lastResult = null;

        const expressionEl = container.querySelector('.calc-expression');
        const resultEl = container.querySelector('.calc-result');

        // Helper: Format number for display
        const formatNumber = (num) => {
            if (typeof num !== 'number' || isNaN(num)) return 'Error';
            if (!isFinite(num)) return num > 0 ? '∞' : '-∞';
            return parseFloat(num.toFixed(10)).toString();
        };

        // Update display
        const updateDisplay = () => {
            resultEl.textContent = currentValue.length > 15 ? parseFloat(currentValue).toExponential(5) : currentValue;
            if (previousValue && operator) {
                const opSymbol = { '+': '+', '-': '−', '*': '×', '/': '÷', 'pow': '^' }[operator] || operator;
                expressionEl.textContent = `${previousValue} ${opSymbol}`;
            } else {
                expressionEl.textContent = '';
            }
        };

        // Calculate result
        const calculate = () => {
            if (!previousValue || !operator) return;
            const prev = parseFloat(previousValue);
            const curr = parseFloat(currentValue);
            let result;

            switch (operator) {
                case '+': result = prev + curr; break;
                case '-': result = prev - curr; break;
                case '*': result = prev * curr; break;
                case '/': result = curr !== 0 ? prev / curr : NaN; break;
                case 'pow': result = Math.pow(prev, curr); break;
                default: return;
            }

            currentValue = formatNumber(result);
            lastResult = result;
            previousValue = '';
            operator = '';
            shouldResetDisplay = true;
        };

        // Apply scientific functions
        const applyScientificFunc = (func) => {
            const num = parseFloat(currentValue);
            let result;

            switch (func) {
                case 'sin': result = Math.sin(num * Math.PI / 180); break;
                case 'cos': result = Math.cos(num * Math.PI / 180); break;
                case 'tan':
                    if (Math.abs(num % 180) === 90) {
                        result = NaN;
                    } else {
                        result = Math.tan(num * Math.PI / 180);
                    }
                    break;
                case 'log': result = num > 0 ? Math.log10(num) : NaN; break;
                case 'ln': result = num > 0 ? Math.log(num) : NaN; break;
                case 'sqrt': result = num >= 0 ? Math.sqrt(num) : NaN; break;
                case 'sq': result = num * num; break;
                case 'inv': result = num !== 0 ? 1 / num : NaN; break;
                case 'abs': result = Math.abs(num); break;
                default: return;
            }

            currentValue = formatNumber(result);
            lastResult = result;
            shouldResetDisplay = true;
            updateDisplay();
        };

        // Button click handlers
        container.querySelectorAll('.calc-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const value = btn.dataset.value;

                // Clear
                if (value === 'C') {
                    currentValue = '0';
                    previousValue = '';
                    operator = '';
                    lastResult = null;
                    updateDisplay();
                    return;
                }

                // Backspace
                if (value === 'backspace') {
                    if (currentValue.length > 1) {
                        currentValue = currentValue.slice(0, -1);
                    } else {
                        currentValue = '0';
                    }
                    updateDisplay();
                    return;
                }

                // Toggle sign
                if (value === '±') {
                    currentValue = (parseFloat(currentValue) * -1).toString();
                    updateDisplay();
                    return;
                }

                // Percent
                if (value === '%') {
                    currentValue = formatNumber(parseFloat(currentValue) / 100);
                    updateDisplay();
                    return;
                }

                // Constants
                if (value === 'pi') {
                    currentValue = Math.PI.toString();
                    shouldResetDisplay = true;
                    updateDisplay();
                    return;
                }
                if (value === 'e') {
                    currentValue = Math.E.toString();
                    shouldResetDisplay = true;
                    updateDisplay();
                    return;
                }

                // Parentheses
                if (value === '(' || value === ')') {
                    if (shouldResetDisplay && value === '(') {
                        currentValue = '(';
                        shouldResetDisplay = false;
                    } else {
                        currentValue += value;
                    }
                    updateDisplay();
                    return;
                }

                // Scientific functions
                if (['sin', 'cos', 'tan', 'log', 'ln', 'sqrt', 'sq', 'inv', 'abs'].includes(value)) {
                    applyScientificFunc(value);
                    return;
                }

                // Power operator
                if (value === 'pow') {
                    if (previousValue && operator) {
                        calculate();
                    }
                    previousValue = currentValue;
                    operator = 'pow';
                    shouldResetDisplay = true;
                    updateDisplay();
                    return;
                }

                // Basic operators
                if (['+', '-', '*', '/'].includes(value)) {
                    if (previousValue && operator) {
                        calculate();
                    }
                    previousValue = currentValue;
                    operator = value;
                    shouldResetDisplay = true;
                    updateDisplay();
                    return;
                }

                // Equals
                if (value === '=') {
                    calculate();
                    updateDisplay();
                    return;
                }

                // Number or decimal
                if (shouldResetDisplay) {
                    currentValue = value === '.' ? '0.' : value;
                    shouldResetDisplay = false;
                } else {
                    if (value === '.' && currentValue.includes('.')) return;
                    currentValue = currentValue === '0' && value !== '.' ? value : currentValue + value;
                }
                updateDisplay();
            });
        });

        // Close button
        container.querySelector('.close-widget-btn-icon').addEventListener('click', () => container.remove());

        return container;
    }
}

// Export to window for global access
window.CalculatorWidget = CalculatorWidget;
