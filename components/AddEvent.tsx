import {
    Pressable,
    StyleSheet,
    Text,
    TextInput,
    View,
    useColorScheme,
    useWindowDimensions,
} from "react-native";
import { Event } from "../types/Event";
import DatePicker from "react-native-date-picker";
import React, { useState } from "react";
import { getButtonSize } from "../utils/Sizes";
import { getTextColor } from "../utils/ColorUtils";
import { Theme } from "../types/Theme";

type RepeatButtonProps = {
    day: string;
    enabled: boolean;
    setEnabled: (enabled: boolean) => void;
};

const RepeatButton: React.FC<RepeatButtonProps> = ({
    day,
    enabled,
    setEnabled,
}) => {
    const { height, width } = useWindowDimensions();

    return (
        <Pressable
            onPress={() => setEnabled(!enabled)}
            style={{
                ...styles.repeatButton,
                borderWidth: enabled ? 1 : 0,
                height: getButtonSize(width),
                width: getButtonSize(width),
            }}
        >
            <Text style={{ ...styles.buttonText, fontSize: width / 20 }}>
                {day.toString()}
            </Text>
        </Pressable>
    );
};

type ColorSelectProps = {
    color: string;
    selected: boolean;
    onSelect: () => void;
};

export const ColorSelect: React.FC<ColorSelectProps> = ({
    color,
    selected,
    onSelect,
}) => {
    const { height, width } = useWindowDimensions();
    const colorScheme = useColorScheme();

    return (
        <Pressable
            onPress={onSelect}
            android_ripple={{ color: "black", borderless: true }}
            style={{
                ...styles.colorSelect,
                backgroundColor: color,
                borderColor: colorScheme === "dark" ? "white" : "black",
                height: getButtonSize(width),
                width: getButtonSize(width),
            }}
        >
            <Text
                style={{
                    ...styles.buttonText,
                    fontSize: width / 20,
                    color: getTextColor(color),
                }}
            >
                {selected ? "✓" : " "}
            </Text>
        </Pressable>
    );
};

type AddEventProps = {
    theme: Theme;
    onAddEvent: (date: Date, event: Event) => void;
    onCancel: () => void;
};

const AddEvent: React.FC<AddEventProps> = ({ theme, onAddEvent, onCancel }) => {
    const [date, setDate] = useState(new Date());
    const [event, setEvent] = useState<Event>({ name: "", color: 0, id: "" });

    const [weekdays, setWeekdays] = useState([
        false,
        false,
        false,
        false,
        false,
        false,
        false,
    ]);

    return (
        <View style={styles.container}>
            <DatePicker date={date} onDateChange={setDate} mode='date' />
            <View style={styles.row}>
                {["M", "T", "W", "T", "F", "S", "S"].map((day, index) => (
                    <RepeatButton
                        key={index}
                        day={day}
                        enabled={weekdays[index]}
                        setEnabled={(enabled) => {
                            let newWeekdays = weekdays.slice();
                            newWeekdays[index] = enabled;
                            setWeekdays(newWeekdays);
                        }}
                    />
                ))}
            </View>
            <View style={styles.row}>
                {theme.colors.map((color, i) => (
                    <ColorSelect
                        key={i}
                        color={color}
                        onSelect={() => setEvent({ ...event, color: i })}
                        selected={event.color === i}
                    />
                ))}
            </View>
            <TextInput
                style={styles.input}
                placeholder='Event name'
                value={event.name}
                onChangeText={(text) => setEvent({ ...event, name: text })}
                onSubmitEditing={() => onAddEvent(date, event)}
            />
        </View>
    );
};

const styles = StyleSheet.create({
    buttonText: {},
    colorSelect: {
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
    },
    container: {
        flex: 1,
        justifyContent: "center",
        alignItems: "center",
    },
    input: {
        height: 40,
        width: "80%",
        borderColor: "gray",
        borderWidth: 1,
        margin: 10,
    },
    repeatButton: {
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
    },
    row: {
        flexDirection: "row",
    },
});

export default AddEvent;
