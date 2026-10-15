#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

namespace {
// Pin assignments and timing.
constexpr uint16_t PWM_TOP = 799;

constexpr uint8_t LEFT_RPWM = 9;
constexpr uint8_t LEFT_LPWM = 10;
constexpr uint8_t LEFT_REN = 4;
constexpr uint8_t LEFT_LEN = 5;

constexpr uint8_t RIGHT_RPWM = 3;
constexpr uint8_t RIGHT_LPWM = 11;
constexpr uint8_t RIGHT_REN = 6;
constexpr uint8_t RIGHT_LEN = 7;

constexpr uint8_t LEFT_OPEN_LIMIT = A0;
constexpr uint8_t LEFT_CLOSED_LIMIT = A1;
constexpr uint8_t RIGHT_OPEN_LIMIT = A2;
constexpr uint8_t RIGHT_CLOSED_LIMIT = A3;
constexpr uint8_t EMERGENCY_STOP = 2;
constexpr uint8_t REMOTE_TRIGGER = 8;
constexpr uint8_t BUZZER_PIN = 13;
constexpr uint8_t LCD_I2C_ADDRESS = 0x27;
constexpr char FIRMWARE_VERSION[] = "v0.4";

constexpr uint16_t MOTOR_DUTY = 700;
constexpr uint8_t RIGHT_MOTOR_DUTY = static_cast<uint8_t>(
	(MOTOR_DUTY * 255UL + PWM_TOP / 2) / PWM_TOP);
constexpr unsigned long STATUS_INTERVAL_MS = 1000;
constexpr unsigned long REMOTE_DEBOUNCE_MS = 50;
constexpr unsigned long CLOSE_DELAY_MS = 30000;
constexpr unsigned long INTERRUPTED_CLOSE_TIMEOUT_MS = 180000;
constexpr unsigned long STARTUP_SPLASH_DURATION_MS = 2000;
constexpr unsigned long BUZZER_DURATION_MS = 3000;
constexpr unsigned long BUZZER_HALF_PERIOD_US = 1000;
// Tune to exceed the slowest full-stroke time measured under load.
constexpr unsigned long MAX_TRAVEL_MS = 60000;

// Controller state.
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);
bool closeCountdownArmed = false;
unsigned long closeCountdownStarted = 0;
unsigned long lastCountdownSecond = 0;
bool closedStatusDisplayed = false;
bool emergencyStopDisplayed = false;
bool emergencyStopWasActive = false;
bool emergencyStopCloseArmed = false;
unsigned long emergencyStopReleasedAt = 0;
bool travelFaultActive = false;
bool travelFaultDisplayed = false;
unsigned long buzzerStartedAt = 0;
unsigned long nextBuzzerToggleAt = 0;
bool buzzerState = false;

enum class Direction : uint8_t { Stopped, Opening, Closing };
Direction pausedRemoteDirection = Direction::Stopped;
unsigned long remotePauseStartedAt = 0;

struct Gate {
	uint8_t rpwm;
	uint8_t lpwm;
	uint8_t ren;
	uint8_t len;
	uint8_t openLimit;
	uint8_t closedLimit;
	Direction direction;
	unsigned long movementStartedAt;

	Gate(uint8_t rpwmPin, uint8_t lpwmPin, uint8_t renPin, uint8_t lenPin,
			 uint8_t openLimitPin, uint8_t closedLimitPin)
			: rpwm(rpwmPin),
				lpwm(lpwmPin),
				ren(renPin),
				len(lenPin),
				openLimit(openLimitPin),
				closedLimit(closedLimitPin),
				direction(Direction::Stopped),
				movementStartedAt(0) {}
};

Gate leftActuator{LEFT_RPWM, LEFT_LPWM, LEFT_REN, LEFT_LEN,
					 LEFT_OPEN_LIMIT, LEFT_CLOSED_LIMIT};
Gate rightActuator{RIGHT_RPWM, RIGHT_LPWM, RIGHT_REN, RIGHT_LEN,
					 RIGHT_OPEN_LIMIT, RIGHT_CLOSED_LIMIT};

// Hardware inputs and motor outputs.
bool limitActive(uint8_t pin) {
	return digitalRead(pin) == LOW;
}

bool emergencyStopActive() {
	return limitActive(EMERGENCY_STOP);
}

void configurePwm() {
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
	ICR1 = PWM_TOP;
	OCR1A = 0;
	OCR1B = 0;

	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
	TCCR2B = _BV(CS20);
	OCR2A = 0;
	OCR2B = 0;
}

void stopBuzzers() {
	digitalWrite(BUZZER_PIN, LOW);
	buzzerState = false;
	buzzerStartedAt = 0;
}

void playStartupTone(uint16_t frequencyHz, uint16_t durationMs) {
	unsigned long halfPeriodUs = 500000UL / frequencyHz;
	unsigned long startedAt = micros();
	unsigned long nextToggleAt = startedAt;
	bool toneState = false;

	while (micros() - startedAt < static_cast<unsigned long>(durationMs) * 1000UL) {
		unsigned long now = micros();
		if (now - nextToggleAt >= halfPeriodUs) {
			toneState = !toneState;
			digitalWrite(BUZZER_PIN, toneState ? HIGH : LOW);
			nextToggleAt += halfPeriodUs;
		}
	}

	digitalWrite(BUZZER_PIN, LOW);
}

void playStartupChime() {
	const uint16_t frequencies[] = {880, 1175, 1568};
	for (uint8_t i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); ++i) {
		playStartupTone(frequencies[i], 140);
		delay(80);
	}
}

void stopGate(Gate& gate) {
	if (gate.rpwm == LEFT_RPWM) {
		OCR1A = 0;
		OCR1B = 0;
	} else {
		OCR2A = 0;
		OCR2B = 0;
	}

	digitalWrite(gate.ren, LOW);
	digitalWrite(gate.len, LOW);
	gate.direction = Direction::Stopped;
	gate.movementStartedAt = 0;
}

bool driveGate(Gate& gate, Direction direction) {
	if (emergencyStopActive() || travelFaultActive) {
		stopGate(gate);
		return false;
	}

	if ((direction == Direction::Opening && limitActive(gate.openLimit)) ||
			(direction == Direction::Closing && limitActive(gate.closedLimit))) {
		stopGate(gate);
		return false;
	}

	stopGate(gate);
	digitalWrite(gate.ren, HIGH);
	digitalWrite(gate.len, HIGH);

	if (gate.rpwm == LEFT_RPWM) {
		if (direction == Direction::Opening) {
			OCR1A = MOTOR_DUTY;
		} else {
			OCR1B = MOTOR_DUTY;
		}
	} else if (direction == Direction::Opening) {
		OCR2B = RIGHT_MOTOR_DUTY;
	} else {
		OCR2A = RIGHT_MOTOR_DUTY;
	}

	gate.direction = direction;
	gate.movementStartedAt = millis();
	closedStatusDisplayed = false;
	return true;
}

void stopAll() {
	closeCountdownArmed = false;
	closeCountdownStarted = 0;
	stopGate(leftActuator);
	stopGate(rightActuator);
	stopBuzzers();
}

uint8_t driveBoth(Direction direction) {
	if (emergencyStopActive() || travelFaultActive) {
		return 0;
	}

	if (direction == Direction::Opening) {
		closeCountdownArmed = true;
		closeCountdownStarted = 0;
	} else if (direction == Direction::Closing) {
		closeCountdownArmed = false;
		closeCountdownStarted = 0;
	}
	uint8_t startedCount = 0;
	if (driveGate(leftActuator, direction)) {
		++startedCount;
	}
	if (driveGate(rightActuator, direction)) {
		++startedCount;
	}
	if (startedCount > 0) {
		buzzerStartedAt = millis();
	}
	return startedCount;
}

bool eitherGateMoving() {
	return leftActuator.direction != Direction::Stopped ||
				 rightActuator.direction != Direction::Stopped;
}

bool bothGatesOpen() {
	return limitActive(leftActuator.openLimit) && limitActive(rightActuator.openLimit);
}

bool bothGatesClosed() {
	return limitActive(leftActuator.closedLimit) && limitActive(rightActuator.closedLimit);
}

// Movement warning buzzer.
void updateBuzzers() {
	if (!eitherGateMoving() || millis() - buzzerStartedAt >= BUZZER_DURATION_MS) {
		stopBuzzers();
		return;
	}

	unsigned long now = micros();
	if (static_cast<long>(now - nextBuzzerToggleAt) >= 0) {
		buzzerState = !buzzerState;
		digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
		nextBuzzerToggleAt = now + BUZZER_HALF_PERIOD_US;
	}
}

// LCD status display.
void printUppercaseText(const char* text) {
	while (*text != '\0') {
		char character = *text++;
		if (character >= 'a' && character <= 'z') {
			character = static_cast<char>(character - 'a' + 'A');
		}
		lcd.write(static_cast<uint8_t>(character));
	}
}

void showMessage(const char* line1, const char* line2) {
	emergencyStopDisplayed = false;
	travelFaultDisplayed = false;
	lcd.clear();
	lcd.setCursor(0, 0);
	printUppercaseText(line1);
	lcd.setCursor(0, 1);
	printUppercaseText(line2);
}

void showClosedStatus() {
	closedStatusDisplayed = true;
	showMessage("CLOSED", "READY");
}

void showOpeningStatus() {
	closedStatusDisplayed = false;
	showMessage("OPENING.. STAY", "CLEAR OF GATE");
}

void showClosingStatus() {
	closedStatusDisplayed = false;
	showMessage("CLOSING.. STAY ", "CLEAR OF GATE");
}

void showEmergencyStopStatus() {
	closedStatusDisplayed = false;
	showMessage("E-STOP ACTIVE", "ALL STOPPED");
	emergencyStopDisplayed = true;
}

void showTravelFaultStatus() {
	closedStatusDisplayed = false;
	showMessage("TRAVEL TIMEOUT", "RESTART ARDUINO");
	travelFaultDisplayed = true;
}

void showCloseCountdown(unsigned long secondsRemaining) {
	char countdownLine[17];
	snprintf(countdownLine, sizeof(countdownLine), "CLOSING IN %2lu", secondsRemaining);
	showMessage(countdownLine, "PRESS TO CLOSE");
}

// Motion requests and RF button handling.
void commandMotion(Direction direction) {
	uint8_t startedCount = driveBoth(direction);
	if (emergencyStopActive()) {
		showEmergencyStopStatus();
		Serial.println(F("Blocked: E-stop active"));
		return;
	}
	if (travelFaultActive) {
		showTravelFaultStatus();
		Serial.println(F("Blocked: travel fault; inspect then restart Arduino"));
		return;
	}
	if (startedCount == 0) {
		if (direction == Direction::Opening && bothGatesOpen()) {
			showCloseCountdown(30);
			Serial.println(F("Already open; close countdown armed"));
		} else if (direction == Direction::Closing && bothGatesClosed()) {
			showClosedStatus();
			Serial.println(F("Already closed"));
		} else {
			showMessage("LIMIT ACTIVE", "NO MOTION");
			Serial.println(F("Blocked: target limit active"));
		}
		return;
	}
	if (startedCount == 1) {
		showMessage(direction == Direction::Opening ? "ONE GATE OPENING" : "ONE GATE CLOSING",
					"CHECK LIMIT SW");
		if (direction == Direction::Opening) {
			Serial.println(F("Partial opening: one gate is at its limit"));
		} else {
			Serial.println(F("Partial closing: one gate is at its limit"));
		}
		return;
	}

	if (direction == Direction::Opening) {
		showOpeningStatus();
		Serial.println(F("Opening"));
	} else {
		showClosingStatus();
		Serial.println(F("Closing"));
	}
}

void beginClosing() {
	commandMotion(Direction::Closing);
}

void printStatus() {
	Serial.print(F("E-stop="));
	Serial.print(emergencyStopActive() ? F("ACTIVE") : F("OK"));
	Serial.print(F(" LEFT="));
	Serial.print(static_cast<uint8_t>(leftActuator.direction));
	Serial.print(F(" RIGHT="));
	Serial.println(static_cast<uint8_t>(rightActuator.direction));
	Serial.print(F("Travel fault="));
	Serial.println(travelFaultActive ? F("ACTIVE") : F("NONE"));

	if (emergencyStopActive()) {
		showEmergencyStopStatus();
	} else if (travelFaultActive) {
		showTravelFaultStatus();
	} else {
		showMessage("System OK",
					leftActuator.direction == Direction::Stopped && rightActuator.direction == Direction::Stopped
						? "Left/right stopped"
						: "Left/right moving");
	}
}

void processCommandLine(const char* command) {
	if (strcmp(command, "OPEN") == 0) {
		pausedRemoteDirection = Direction::Stopped;
		remotePauseStartedAt = 0;
		commandMotion(Direction::Opening);
	} else if (strcmp(command, "CLOSE") == 0) {
		pausedRemoteDirection = Direction::Stopped;
		remotePauseStartedAt = 0;
		commandMotion(Direction::Closing);
	} else if (strcmp(command, "STOP") == 0) {
		pausedRemoteDirection = Direction::Stopped;
		remotePauseStartedAt = 0;
		stopAll();
		if (emergencyStopActive()) {
			showEmergencyStopStatus();
		} else if (travelFaultActive) {
			showTravelFaultStatus();
		} else {
			showMessage("STOPPED", "PRESS TO OPEN");
		}
		Serial.println(F("Stopped"));
	} else if (strcmp(command, "STATUS") == 0) {
		printStatus();
	} else if (strcmp(command, "RESET") == 0) {
		if (emergencyStopActive()) {
			showEmergencyStopStatus();
			Serial.println(F("Reset refused: E-stop active"));
		} else if (eitherGateMoving()) {
			Serial.println(F("Reset refused: stop motion first"));
		} else if (travelFaultActive) {
			travelFaultActive = false;
			travelFaultDisplayed = false;
			showMessage("FAULT CLEARED", "CHECK BEFORE RUN");
			Serial.println(F("Travel fault cleared; inspect gate and limit switches"));
		} else {
			Serial.println(F("No travel fault to reset"));
		}
	} else {
		Serial.println(F("Commands: OPEN, CLOSE, STOP, STATUS, RESET"));
	}
}

void processCommand() {
	static char commandBuffer[16];
	static uint8_t commandLength = 0;
	static bool commandOverflow = false;

	while (Serial.available() > 0) {
		char character = static_cast<char>(Serial.read());
		if (character == '\r' || character == '\n') {
			if (commandOverflow) {
				Serial.println(F("Command too long"));
			} else if (commandLength > 0) {
				commandBuffer[commandLength] = '\0';
				processCommandLine(commandBuffer);
			}
			commandLength = 0;
			commandOverflow = false;
			continue;
		}

		if (commandOverflow) {
			continue;
		}
		if (commandLength >= sizeof(commandBuffer) - 1) {
			commandOverflow = true;
			continue;
		}
		if (character >= 'a' && character <= 'z') {
			character = static_cast<char>(character - 'a' + 'A');
		}
		commandBuffer[commandLength++] = character;
	}
}

void handleRemoteTrigger() {
	static bool lastSignal = digitalRead(REMOTE_TRIGGER);
	static unsigned long lastChange = 0;
	bool signal = digitalRead(REMOTE_TRIGGER);
	unsigned long now = millis();

	if (signal != lastSignal && now - lastChange >= REMOTE_DEBOUNCE_MS) {
		lastSignal = signal;
		lastChange = now;
		if (signal == LOW) {
			if (emergencyStopCloseArmed) {
				emergencyStopCloseArmed = false;
				pausedRemoteDirection = Direction::Stopped;
				remotePauseStartedAt = 0;
				Serial.println(F("RF press; closing without E-stop release delay"));
				beginClosing();
			} else if (eitherGateMoving()) {
				pausedRemoteDirection =
					leftActuator.direction != Direction::Stopped
						? leftActuator.direction
						: rightActuator.direction;
				remotePauseStartedAt = now;
				stopAll();
				showMessage(pausedRemoteDirection == Direction::Opening
								? "PAUSED OPENING"
								: "PAUSED CLOSING",
							"PRESS TO RESUME");
			} else if (pausedRemoteDirection != Direction::Stopped) {
				if (emergencyStopActive() || travelFaultActive) {
					remotePauseStartedAt = now;
					commandMotion(pausedRemoteDirection);
				} else {
					Direction resumeDirection = pausedRemoteDirection;
					pausedRemoteDirection = Direction::Stopped;
					remotePauseStartedAt = 0;
					commandMotion(resumeDirection);
				}
			} else if (bothGatesOpen()) {
				closeCountdownArmed = false;
				closeCountdownStarted = 0;
				commandMotion(Direction::Closing);
			} else {
				commandMotion(Direction::Opening);
			}
		}
	}
}

void updateEmergencyStopReleaseTimer() {
	bool active = emergencyStopActive();
	if (active) {
		emergencyStopWasActive = true;
		emergencyStopCloseArmed = false;
		return;
	}

	if (emergencyStopWasActive) {
		emergencyStopWasActive = false;
		emergencyStopCloseArmed = true;
		emergencyStopReleasedAt = millis();
	}
}

void updateEmergencyStopClose() {
	if (!emergencyStopCloseArmed || emergencyStopActive()) {
		return;
	}

	if (bothGatesClosed()) {
		emergencyStopCloseArmed = false;
		return;
	}

	if (millis() - emergencyStopReleasedAt < INTERRUPTED_CLOSE_TIMEOUT_MS) {
		return;
	}

	emergencyStopCloseArmed = false;
	pausedRemoteDirection = Direction::Stopped;
	remotePauseStartedAt = 0;
	Serial.println(F("E-stop release timeout; closing"));
	beginClosing();
}

void updatePausedRemoteTimeout() {
	if (pausedRemoteDirection == Direction::Stopped) {
		return;
	}

	if (bothGatesClosed()) {
		pausedRemoteDirection = Direction::Stopped;
		remotePauseStartedAt = 0;
		return;
	}

	if (emergencyStopActive() || travelFaultActive ||
			millis() - remotePauseStartedAt < INTERRUPTED_CLOSE_TIMEOUT_MS) {
		return;
	}

	pausedRemoteDirection = Direction::Stopped;
	remotePauseStartedAt = 0;
	Serial.println(F("Paused motion timeout; closing"));
	beginClosing();
}

// Safety enforcement and automatic movement timers.
void enforceSafety() {
	if (emergencyStopActive()) {
		stopAll();
		if (!emergencyStopDisplayed) {
			showEmergencyStopStatus();
			Serial.println(F("E-stop active; all motion stopped"));
		}
		return;
	}
	emergencyStopDisplayed = false;

	if (travelFaultActive) {
		if (!travelFaultDisplayed) {
			showTravelFaultStatus();
		}
		return;
	}
	travelFaultDisplayed = false;

	if (leftActuator.direction == Direction::Opening && limitActive(leftActuator.openLimit)) {
		stopGate(leftActuator);
	} else if (leftActuator.direction == Direction::Closing && limitActive(leftActuator.closedLimit)) {
		stopGate(leftActuator);
	}

	if (rightActuator.direction == Direction::Opening && limitActive(rightActuator.openLimit)) {
		stopGate(rightActuator);
	} else if (rightActuator.direction == Direction::Closing && limitActive(rightActuator.closedLimit)) {
		stopGate(rightActuator);
	}

	unsigned long now = millis();
	bool leftTimedOut = leftActuator.direction != Direction::Stopped &&
						now - leftActuator.movementStartedAt >= MAX_TRAVEL_MS;
	bool rightTimedOut = rightActuator.direction != Direction::Stopped &&
						 now - rightActuator.movementStartedAt >= MAX_TRAVEL_MS;
	if (leftTimedOut || rightTimedOut) {
		travelFaultActive = true;
		stopAll();
		showTravelFaultStatus();
		Serial.println(F("Travel timeout; motion stopped. Inspect, then restart Arduino"));
		return;
	}

	if (bothGatesClosed() && !closedStatusDisplayed) {
		showClosedStatus();
		Serial.println(F("closed."));
	}
}

void updateAutomaticClose() {
	if (!closeCountdownArmed || !bothGatesOpen()) {
		return;
	}

	unsigned long now = millis();
	if (closeCountdownStarted == 0) {
		closeCountdownStarted = now;
		lastCountdownSecond = 30;
		showCloseCountdown(30);
		Serial.println(F("Closing in 30"));
	}

	unsigned long elapsed = now - closeCountdownStarted;
	if (elapsed >= CLOSE_DELAY_MS) {
		closeCountdownArmed = false;
		closeCountdownStarted = 0;
		beginClosing();
		return;
	}

	unsigned long secondsRemaining = (CLOSE_DELAY_MS - elapsed + 999) / 1000;
	if (secondsRemaining != lastCountdownSecond) {
		lastCountdownSecond = secondsRemaining;
		showCloseCountdown(secondsRemaining);
		Serial.print(F("Closing in "));
		Serial.println(secondsRemaining);
	}
}

}

// Arduino lifecycle.
void setup() {
	pinMode(LEFT_RPWM, OUTPUT);
	pinMode(LEFT_LPWM, OUTPUT);
	pinMode(LEFT_REN, OUTPUT);
	pinMode(LEFT_LEN, OUTPUT);
	pinMode(RIGHT_RPWM, OUTPUT);
	pinMode(RIGHT_LPWM, OUTPUT);
	pinMode(RIGHT_REN, OUTPUT);
	pinMode(RIGHT_LEN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	pinMode(LEFT_OPEN_LIMIT, INPUT_PULLUP);
	pinMode(LEFT_CLOSED_LIMIT, INPUT_PULLUP);
	pinMode(RIGHT_OPEN_LIMIT, INPUT_PULLUP);
	pinMode(RIGHT_CLOSED_LIMIT, INPUT_PULLUP);
	pinMode(EMERGENCY_STOP, INPUT_PULLUP);
	pinMode(REMOTE_TRIGGER, INPUT_PULLUP);

	emergencyStopWasActive = emergencyStopActive();
	configurePwm();
	stopAll();
	Serial.begin(115200);
	lcd.init();
	lcd.backlight();
	unsigned long splashStartedAt = millis();
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(F("North Gate v0.4"));
	lcd.setCursor(0, 1);
	lcd.print(F("-Nathan Rasbold"));
	playStartupChime();
	unsigned long splashElapsed = millis() - splashStartedAt;
	if (splashElapsed < STARTUP_SPLASH_DURATION_MS) {
		delay(STARTUP_SPLASH_DURATION_MS - splashElapsed);
	}
	showMessage("POSITION UNKNOWN", "PRESS REMOTE");
	Serial.print(F("North gate controller "));
	Serial.print(FIRMWARE_VERSION);
	Serial.println(F(" ready"));
	Serial.println(F("Remote: press to pause/resume motion"));
	Serial.println(F("Serial commands: OPEN, CLOSE, STOP, STATUS, RESET"));
}

void loop() {
	// Read requests before enforcing limits and automatic movement.
	updateEmergencyStopReleaseTimer();
	handleRemoteTrigger();
	processCommand();
	enforceSafety();
	updateAutomaticClose();
	updateEmergencyStopClose();
	updatePausedRemoteTimeout();
	updateBuzzers();

	// Periodic diagnostic output.
	static unsigned long lastStatus = 0;
	if (millis() - lastStatus >= STATUS_INTERVAL_MS) {
		lastStatus = millis();
		if (emergencyStopActive()) {
			Serial.println(F("E-stop active"));
		}
	}
}
