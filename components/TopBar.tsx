import { useState } from "react";
import {
    Button,
    Pressable,
    StyleSheet,
    Text,
    View,
    useColorScheme,
    useWindowDimensions,
} from "react-native";
import { getButtonSize } from "../utils/Sizes";
import { getTextColor } from "../utils/ColorUtils";

type ColorSwitchProps = {
    color: string;
};

export const ColorSwitch: React.FC<ColorSwitchProps> = ({ color }) => {
    const [isEnabled, setIsEnabled] = useState(true);

    const toggleSwitch = () => {
        setIsEnabled(!isEnabled);
    };

    const { height, width } = useWindowDimensions();
    const colorScheme = useColorScheme();

    return (
        <Pressable
            onPress={toggleSwitch}
            android_ripple={{ color: "black", borderless: true }}
            style={{
                ...styles.colorSwitch,
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
                {isEnabled ? "✓" : " "}
            </Text>
        </Pressable>
    );
};

type TopBarProps = {
    colors: string[];
};

const TopBar: React.FC<TopBarProps> = ({ colors }) => {
    const colorScheme = useColorScheme();

    return (
        <View
            style={{
                ...styles.topBar,
                backgroundColor: colorScheme === "dark" ? "black" : "white",
            }}
        >
            {colors.map((color, index) => (
                <ColorSwitch key={index} color={color} />
            ))}
        </View>
    );
};

const styles = StyleSheet.create({
    buttonText: {},
    colorSwitch: {
        borderRadius: "50%",
        borderWidth: 1,
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        cursor: "pointer",
        marginLeft: 10,
        marginBottom: 10,
    },
    topBar: {
        display: "flex",
        // justifyContent: "space-between",
        flexDirection: "row",
        width: "100%",
    },
});

export default TopBar;
