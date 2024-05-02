function hexToRgb(hex: string): number[] | null {
    const match = hex.match(/^#([a-f0-9]{6})$/i);
    if (!match) {
        return null;
    }

    const hexColor = match[1];
    return [
        parseInt(hexColor.substring(0, 2), 16),
        parseInt(hexColor.substring(2, 4), 16),
        parseInt(hexColor.substring(4, 6), 16),
    ];
}

function rgbToLuminance(rgb: number[]): number {
    const [r, g, b] = rgb.map((c) => {
        c /= 255;
        return c <= 0.03928 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4);
    });

    return r * 0.2126 + g * 0.7152 + b * 0.0722;
}

export function getLuminance(color: string): number {
    const rgb = hexToRgb(color);
    return rgb ? rgbToLuminance(rgb) : 0;
}

export function getTextColor(color: string): string {
    return getLuminance(color) > 0.2 ? "black" : "white";
}
