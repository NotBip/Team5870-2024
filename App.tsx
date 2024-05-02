import { StatusBar } from "expo-status-bar";

import TopBar from "./components/TopBar";
import EventList from "./components/EventList";
import AddEvent from "./components/AddEvent";
import SideBar from "./components/SideBar";

import {
    SafeAreaProvider,
    useSafeAreaInsets,
} from "react-native-safe-area-context";

import { StyleSheet, Text, View, useColorScheme } from "react-native";
import { EventsOnDate } from "./types/Event";
import { useState } from "react";

/**
 * Remember is a to-do list app that helps you organize your tasks and stay productive.
 *
 */

const MainView: React.FC = () => {
    const testTheme = {
        name: "Test Theme",
        colors: [
            "#ffffff",
            "#ff0000",
            "#ff8000",
            "#ffff00",
            "#00ff00",
            "#00ffff",
            "#0000ff",
            "#000000",
        ],
    };

    const testEvents: EventsOnDate[] = [
        {
            date: new Date(),
            events: [
                {
                    id: "1",
                    name: "Poop",
                    color: 0,
                },
                {
                    id: "2",
                    name: "Apply to jobs",
                    color: 1,
                },
                {
                    id: "3",
                    name: "Poop again",
                    color: 2,
                },
            ],
        },
        {
            date: new Date(),
            events: [
                {
                    id: "4",
                    name: "Respectfully disagree with someone",
                    color: 1,
                },
                {
                    id: "5",
                    name: "Take a nap",
                    color: 0,
                },
                {
                    id: "6",
                    name: "Use the bathroom",
                    color: 0,
                },
            ],
        },
    ];

    const [addingEvent, setAddingEvent] = useState(false);
    const [addingEventDay, setAddingEventDay] = useState(new Date());

    const insets = useSafeAreaInsets();
    const colorScheme = useColorScheme();

    return (
        <View
            style={{
                ...styles.container,
                backgroundColor: colorScheme === "dark" ? "black" : "white",
                paddingTop: insets.top,
                paddingBottom: 0, // Ignore safe area on the bottom
                paddingLeft: insets.left,
                paddingRight: insets.right,
            }}
        >
            <StatusBar style='auto' />
            <TopBar colors={testTheme.colors} />
            <View style={styles.mainView}>
                {addingEvent ? (
                    <AddEvent
                        theme={testTheme}
                        onAddEvent={(event) => {
                            setAddingEvent(false);
                        }}
                        onCancel={() => setAddingEvent(false)}
                    />
                ) : (
                    <></>
                )}
                <EventList events={testEvents} theme={testTheme} />
                <SideBar
                    onDayClick={(day) => {
                        setAddingEvent(true);
                        setAddingEventDay(day);
                    }}
                />
            </View>
        </View>
    );
};

export default function App() {
    return (
        <SafeAreaProvider>
            <MainView />
        </SafeAreaProvider>
    );
}

const styles = StyleSheet.create({
    container: {
        height: "100%",
        width: "100%",
        alignItems: "center",
        justifyContent: "center",
    },
    mainView: {
        flex: 6, // Take up most of the screen vertically
        display: "flex",
        flexDirection: "row",
        width: "100%",
        justifyContent: "space-between",
    },
});
