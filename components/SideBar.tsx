import {
    Button,
    FlatList,
    Pressable,
    StyleSheet,
    Text,
    View,
    useColorScheme,
    useWindowDimensions,
} from "react-native";
import { getButtonSize } from "../utils/Sizes";

type DayButtonProps = {
    day: string;
    onDayClick: () => void;
};

const DayButton: React.FC<DayButtonProps> = ({ day, onDayClick }) => {
    const { height, width } = useWindowDimensions();

    return (
        <Pressable
            onPress={() => onDayClick()}
            style={{
                ...styles.dayButton,
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

type SideBarProps = {
    onDayClick: (day: Date) => void;
};

function getTodayPlusDays(days: number): Date {
    let today = new Date();
    today.setDate(today.getDate() + days);
    return today;
}

const SideBar: React.FC<SideBarProps> = ({ onDayClick }) => {
    // if the current month has 30 days, days is an array of the next 30 Dates
    // if the current month has 31 days, days is an array of the next 31 Dates

    const numDaysInMonth = new Date(
        new Date().getFullYear(),
        new Date().getMonth() + 1,
        0
    ).getDate();

    let weekdays = ["S", "M", "T", "W", "T", "F", "S"];
    // Rotate so that the first item in weekdays is the current day of the week
    let currentDay = new Date().getDay();

    let days: string[] = weekdays
        .slice(currentDay)
        .concat(weekdays.slice(0, currentDay));

    let current = new Date();
    current.setDate(current.getDate() + 7);

    for (let i = 1; i <= numDaysInMonth - 7; i++) {
        days.push(current.getDate().toString());
        current.setDate(current.getDate() + 1);
    }

    const colorScheme = useColorScheme();

    return (
        <View
            style={{
                ...styles.sideBar,
                backgroundColor: colorScheme === "dark" ? "#333" : "lightgrey",
            }}
        >
            <FlatList
                data={days}
                renderItem={({ item, index }) => (
                    <DayButton
                        day={item}
                        onDayClick={() => onDayClick(getTodayPlusDays(index))}
                    />
                )}
                keyExtractor={(item, index) => index.toString()}
            />
        </View>
    );
};

const styles = StyleSheet.create({
    buttonText: {
        fontWeight: "300",
        color: "white",
    },
    dayButton: {
        borderRadius: "50%",
        borderColor: "black",
        borderWidth: 1,
        backgroundColor: "red",
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        cursor: "pointer",
        marginTop: 10,
        marginBottom: 0,
        marginLeft: 10,
        marginRight: 10,
    },
    sideBar: {
        display: "flex",
        flexDirection: "row",
        height: "100%",
        paddingBottom: 10,
    },
});

export default SideBar;
