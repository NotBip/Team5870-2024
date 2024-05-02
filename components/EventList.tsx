import React from "react";
import {
    SectionList,
    StyleSheet,
    Text,
    View,
    useColorScheme,
} from "react-native";

import type { EventsOnDate } from "../types/Event";
import type { Theme } from "../types/Theme";
import EventListItem from "./EventListItem";
import EventListSectionHeader from "./EventListSectionHeader";
import { formatDateString } from "../utils/DateUtils";

type EventListProps = {
    events: EventsOnDate[];
    theme: Theme;
};

const EventList: React.FC<EventListProps> = ({ events, theme }) => {
    const colorScheme = useColorScheme();

    return (
        <View style={styles.container}>
            <SectionList
                style={{
                    ...styles.eventList,
                    backgroundColor: colorScheme === "dark" ? "black" : "white",
                }}
                sections={events.map((eventsOnDate) => ({
                    title: formatDateString(eventsOnDate.date),
                    data: eventsOnDate.events,
                }))}
                keyExtractor={(event, _) => event.id}
                renderItem={({ item }) => (
                    <EventListItem
                        event={item}
                        color={theme.colors[item.color]}
                    />
                )}
                renderSectionHeader={({ section: { title } }) => (
                    <EventListSectionHeader title={title} />
                )}
            />
        </View>
    );
};

const styles = StyleSheet.create({
    container: {
        flex: 6,
        alignItems: "center",
        justifyContent: "center",
        width: "100%",
    },
    eventList: {
        width: "100%",
    },
});

export default EventList;
