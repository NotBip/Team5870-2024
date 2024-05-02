function getPrefix(date: Date): string {
    const today = new Date();
    today.setHours(0, 0, 0, 0);

    const tomorrow = new Date(today);
    tomorrow.setDate(today.getDate() + 1);
    const yesterday = new Date(today);
    yesterday.setDate(today.getDate() - 1);

    const dateNoTime = new Date(date);
    dateNoTime.setHours(0, 0, 0, 0);

    if (dateNoTime.getTime() === tomorrow.getTime()) {
        return "Tomorrow - ";
    }
    if (dateNoTime.getTime() === yesterday.getTime()) {
        return "Yesterday - ";
    }
    if (dateNoTime.getTime() === today.getTime()) {
        return "Today - ";
    }
    return "";
}

function getSuffix(dayOfMonth: number): string {
    if (dayOfMonth === 1 || dayOfMonth === 21 || dayOfMonth === 31) {
        return "st";
    }
    if (dayOfMonth === 2 || dayOfMonth === 22) {
        return "nd";
    }
    if (dayOfMonth === 3 || dayOfMonth === 23) {
        return "rd";
    }
    return "th";
}

export function formatDateString(date: Date): string {
    // Format the day as "Monday, December 31st"
    // Add ", 2025" if the year is not the current year
    // Add "Tommorrow - " if the date is tomorrow, "Today - " if the date is today, and "Yesterday - " if the date is yesterday

    const prefix = getPrefix(date);
    const day = date.toLocaleString("default", { weekday: "long" });
    const month = date.toLocaleString("default", { month: "long" });
    const dayOfMonth = date.getDate();
    const suffix = getSuffix(dayOfMonth);
    const year = date.getFullYear();

    return `${prefix}${day}, ${month} ${dayOfMonth}${suffix}${
        year !== new Date().getFullYear() ? `, ${year}` : ""
    }`;
}
