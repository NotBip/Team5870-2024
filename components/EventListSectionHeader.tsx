import { StyleSheet, Text, View, useColorScheme } from "react-native";

/**
 * A simple component to display an event in the EventList component.
 * Takes in an event object and a color from the theme object.
 */

import type { Event } from "../types/Event";

type EventProps = {
    title: string;
};

const EventListSectionHeader: React.FC<EventProps> = ({ title }) => {
    const colorScheme = useColorScheme();

    return (
        <View
            style={{
                ...styles.eventListSectionHeader,
                backgroundColor: colorScheme === "dark" ? "black" : "white",
                borderBottomColor: colorScheme === "dark" ? "black" : "black",
            }}
        >
            <Text style={styles.sectionHeaderText}>{title}</Text>
        </View>
    );
};

const styles = StyleSheet.create({
    eventListSectionHeader: {
        display: "flex",
        width: "100%",
        borderBottomWidth: 1,
        paddingLeft: 10,
        paddingTop: 20,
        paddingBottom: 10,
    },
    sectionHeaderText: {
        fontWeight: "600",
        color: "gray",
        fontSize: 15,
    },
});

export default EventListSectionHeader;
