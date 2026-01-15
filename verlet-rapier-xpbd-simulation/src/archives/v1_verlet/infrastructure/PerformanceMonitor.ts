import type { Disposable } from '../types';

export class PerformanceMonitor implements Disposable {
    private fps = 60;
    private frameCount = 0;
    private lastTime = performance.now();
    private panel: HTMLDivElement;

    // Quality adjustment callbacks
    private onLowPerformance?: () => void;
    private onHighPerformance?: () => void;

    constructor() {
        this.panel = this.createPanel();
        document.body.appendChild(this.panel);
    }

    private createPanel(): HTMLDivElement {
        const div = document.createElement('div');
        div.style.position = 'absolute';
        div.style.top = '10px';
        div.style.right = '10px';
        div.style.background = 'rgba(0, 0, 0, 0.7)';
        div.style.color = '#00ff00';
        div.style.padding = '5px 10px';
        div.style.fontFamily = 'monospace';
        div.style.fontSize = '12px';
        div.style.pointerEvents = 'none';
        div.innerHTML = 'FPS: --';
        return div;
    }

    update() {
        this.frameCount++;
        const time = performance.now();

        // Update every second
        if (time >= this.lastTime + 1000) {
            this.fps = Math.round((this.frameCount * 1000) / (time - this.lastTime));
            this.panel.innerText = `FPS: ${this.fps}`;

            this.checkThresholds();

            this.lastTime = time;
            this.frameCount = 0;
        }
    }

    private checkThresholds() {
        // Simple adaptive logic (Page 5)
        if (this.fps < 30) {
            this.panel.style.color = 'red';
            this.onLowPerformance?.();
        } else if (this.fps > 55) {
            this.panel.style.color = '#00ff00';
            this.onHighPerformance?.();
        } else {
            this.panel.style.color = 'yellow';
        }
    }

    setCallbacks(onLow: () => void, onHigh: () => void) {
        this.onLowPerformance = onLow;
        this.onHighPerformance = onHigh;
    }

    dispose() {
        if (this.panel.parentNode) {
            this.panel.parentNode.removeChild(this.panel);
        }
    }
}