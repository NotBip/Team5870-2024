import { StyleSheet, Text, View } from "react-native";

/**
 * A simple component to display an event in the EventList component.
 * Takes in an event object and a color from the theme object.
 */

import type { Event } from "../types/Event";
import { getLuminance, getTextColor } from "../utils/ColorUtils";

type EventProps = {
    event: Event;
    color: string;
};

const EventListItem: React.FC<EventProps> = ({ event, color }) => {
    return (
        <View
            style={{
                ...styles.eventListItem,
                backgroundColor: color,
                borderBottomColor:
                    getLuminance(color) > 0.9 ? "lightgrey" : color,
            }}
        >
            <Text
                style={{
                    ...styles.eventListItemText,
                    color: getTextColor(color),
                }}
            >
                {event.name}
            </Text>
        </View>
    );
};

const styles = StyleSheet.create({
    eventListItem: {
        display: "flex",
        padding: 10,
        margin: 0,
        width: "100%",
        // marginBottom: -0.5,
        borderBottomWidth: 1,
        borderBottomColor: "lightgrey",
    },
    eventListItemText: {
        fontSize: 17,
        fontWeight: "400",
    },
});

export default EventListItem;
