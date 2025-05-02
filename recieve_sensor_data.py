import serial
import time
import numpy as np
import joblib # To load sklearn models/scaler/encoder
from tensorflow import keras # To load Keras NN model
import math # For sqrt
import os # To construct file paths reliably
import traceback # For detailed error printing

# --- Configuration ---
SERIAL_PORT = "/dev/serial0"  # Or /dev/ttyS0, /dev/ttyUSB0, etc.
BAUD_RATE = 9600
SERIAL_READ_TIMEOUT = 0.2
SCORE_THRESHOLD = 2
final_score_to_send = 0

PROCESS_INTERVAL = 1  # Seconds

# --- Model Configuration ---
MODEL_DIR = "Model"
SCALER_FILE = os.path.join(MODEL_DIR, 'scaler.pkl')
ENCODER_FILE = os.path.join(MODEL_DIR, 'encoder.pkl')
RF_MODEL_FILE = os.path.join(MODEL_DIR, 'rf_model.pkl')
NN_MODEL_FILE = os.path.join(MODEL_DIR, 'nn_model.keras')
META_MODEL_FILE = os.path.join(MODEL_DIR, 'meta_model.pkl')

# --- Globals ---
interval_data_R = []
interval_data_L = []
last_process_time = time.monotonic()
serial_buffer = ""
n_classes = 0 # Will be determined from loaded encoder/model

# --- Load Models and Preprocessing Objects ---ç
print("Loading models and preprocessing objects...")
try:
    if not os.path.exists(MODEL_DIR):
        raise FileNotFoundError(f"Model directory '{MODEL_DIR}' not found.")
    scaler = joblib.load(SCALER_FILE)
    encoder = joblib.load(ENCODER_FILE)
    rf_model = joblib.load(RF_MODEL_FILE)
    nn_model = keras.models.load_model(NN_MODEL_FILE)
    meta_model = joblib.load(META_MODEL_FILE)
    n_classes = len(encoder.categories_[0])
    print(f"Determined number of RASS classes: {n_classes}")
    if n_classes <= 0: raise ValueError("Could not determine classes from encoder.")
    print("Models and objects loaded successfully.")
except FileNotFoundError as e: print(f"Error loading file: {e}"); exit()
except Exception as e: print(f"Error during model loading: {e}"); traceback.print_exc(); exit()

# --- Serial Port Setup ---
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_READ_TIMEOUT)
    ser.flush()
    print(f"Serial port {ser.name} opened successfully at {BAUD_RATE} baud.")
except serial.SerialException as e: print(f"Error opening serial port {SERIAL_PORT}: {e}"); exit()
except Exception as e: print(f"Error during serial port setup: {e}"); traceback.print_exc(); exit()

# --- Prediction Function ---
def predict_batch_score(data_list, scaler, rf_model, nn_model, meta_model, n_classes):
    """ Predicts sum RASS score for batch using 9 features. """
    if not data_list: return 0
    try:
        # Extract 9 sensor features (indices 1 to 9)
        sensor_data_array = np.array([item[1:10] for item in data_list])
        if sensor_data_array.ndim != 2 or sensor_data_array.shape[0] == 0 or sensor_data_array.shape[1] != 9:
             print(f"Warn: Incorrect data shape for ML: {sensor_data_array.shape}. Expected (?, 9)")
             return 0

        scaled_data = scaler.transform(sensor_data_array)
        rf_preds = rf_model.predict(scaled_data)
        rf_preds = np.clip(rf_preds.astype(int), 0, n_classes - 1)
        rf_preds_one_hot = np.eye(n_classes)[rf_preds]
        nn_preds_proba = nn_model.predict(scaled_data, verbose=0)

        if rf_preds_one_hot.shape[0] != nn_preds_proba.shape[0]: return 0 # Shape mismatch check
        stacked_features = np.hstack([rf_preds_one_hot, nn_preds_proba])
        final_preds_indices = meta_model.predict(stacked_features)
        batch_score = np.sum(final_preds_indices)
        return batch_score
    except Exception as e: print(f"Error during prediction: {e}"); traceback.print_exc(); return 0

# --- Helper Function to Calculate Average Gyro Magnitude ---
def calculate_batch_gyro_magnitude(data_list):
    """ Calculates average gyro magnitude from list of lists. """
    if not data_list: return 0.0
    total_magnitude = 0.0
    num_points = 0
    for data_point in data_list:
        # Gyro data is at indices 1, 2, 3
        if len(data_point) >= 4:
            try:
                gx, gy, gz = float(data_point[1]), float(data_point[2]), float(data_point[3])
                magnitude = math.sqrt(gx**2 + gy**2 + gz**2)
                total_magnitude += magnitude
                num_points += 1
            except (ValueError, TypeError, IndexError): continue # Skip malformed points silently here
        # else: print(f"Warn: Short data point in mag calc: {data_point}") # Reduce noise
    return total_magnitude / num_points if num_points > 0 else 0.0

# --- Processing Function (Calls prediction, calc magnitude, handles TX) ---
def process_and_transmit_results(data_R_list, data_L_list, interval_start_time, ser_conn):
    """ Processes batches, gets ML score, calculates avg magnitudes, sends result. """
    print("=" * 50)
    print(f"Processing Batch - Interval Ended at {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Received {len(data_R_list)} points R, {len(data_L_list)} points L")

    # Make ML predictions
    score_R = predict_batch_score(data_R_list, scaler, rf_model, nn_model, meta_model, n_classes)
    score_L = predict_batch_score(data_L_list, scaler, rf_model, nn_model, meta_model, n_classes)
    total_score = (score_R + score_L)

    # Calculate average magnitudes for the batches
    avg_mag_R = calculate_batch_gyro_magnitude(data_R_list)
    avg_mag_L = calculate_batch_gyro_magnitude(data_L_list)

    print(f"  ML Score: R={score_R}, L={score_L}, Total={total_score}")
    print(f"  Avg Mag:  R={avg_mag_R:.4f}, L={avg_mag_L:.4f}")
    print("=" * 50)

    if abs(total_score) > SCORE_THRESHOLD:
        final_score_to_send = total_score # Send the actual score
        print(f"  Score exceeds threshold ({SCORE_THRESHOLD}).")
    else:
        # Score is below threshold, treat as zero for motor command
        final_score_to_send = 0
        print(f"  Score within threshold ({SCORE_THRESHOLD}), sending 0.")

    # --- Send the combined results back via HC-12 ---
    # Format: "AgResult,TOTAL_SCORE,AVG_MAG_R,AVG_MAG_L\n"
    try:
        if ser_conn and ser_conn.is_open:
            # Format with 4 decimal places for magnitudes
            message_to_send = f"AgResult,{final_score_to_send},{avg_mag_R:.4f},{avg_mag_L:.4f}\n"
            print(f"Serial TX -> {message_to_send.strip()}") # Log sent message
            ser_conn.write(message_to_send.encode('utf-8'))
        else:
            print("Error: Cannot send result, serial port not open.")
    except serial.SerialException as e: print(f"Error writing result to serial port: {e}")
    except Exception as e: print(f"Error formatting or sending result: {e}"); traceback.print_exc()
    # -------------------------------------------------

# --- Main Loop ---
print(f"Starting data reception (ML Mode). Accumulating for {PROCESS_INTERVAL}s intervals.")
print("Expecting format '$SIDE,GX,GY,GZ,AX,AY,AZ,MX,MY,MZ,GyroMag~\\n' (11 parts in content)")
print("Will send 'AgResult,TOTAL_SCORE,AVG_MAG_R,AVG_MAG_L\\n' periodically.")
print("Press Ctrl+C to exit.")

while True:
    try:
        # --- Part 1: Read data into buffer ---
        if ser and ser.is_open:
            if ser.in_waiting > 0:
                try:
                    data_bytes = ser.read(ser.in_waiting)
                    #print(data_bytes)
                    serial_buffer += data_bytes.decode('utf-8', errors='ignore')
                except Exception as e: print(f"Error reading serial: {e}")
        elif not (ser and ser.is_open): # Reopen logic
             print("Serial port closed. Reopening...")
             time.sleep(2)
             try:
                 if ser is None: ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=SERIAL_READ_TIMEOUT)
                 else: ser.open()
                 ser.flush()
                 print("Serial port reopened.")
             except Exception as e: print(f"Failed to reopen: {e}. Exiting."); break

        # --- Part 2: Process complete lines from buffer ---
        while '\n' in serial_buffer:
            line, serial_buffer = serial_buffer.split('\n', 1)
            line = line.strip()
            if not line: continue

            if line.startswith('$') and line.endswith('~'):
                content = line[1:-1]
            else: continue # Skip malformed lines

            # Parse content (expecting 11 parts: SIDE + 9 features + Mag)
            parts = content.split(',')
            if len(parts) == 11: # <<<--- UPDATED LENGTH CHECK
                try:
                    side = parts[0].strip().upper()
                    timestamp = time.monotonic()
                    # All values after side (indices 1 to 10)
                    all_values = [float(p) for p in parts[1:]]

                    if len(all_values) == 10: # Check parsing produced 10 numbers
                        data_point = [timestamp] + all_values # Store timestamp + 10 values
                        if side == 'R':
                            interval_data_R.append(data_point)
                        elif side == 'L':
                            interval_data_L.append(data_point)
                        # else: Unknown side -> ignore silently
                    # else: Incorrect number of values -> ignore silently

                except ValueError: pass # Ignore lines with non-float values silently
                except Exception as e: print(f"Error processing content '{content}': {e}"); traceback.print_exc()
            # else: Ignore lines with wrong number of parts silently unless debugging needed

        # --- Part 3: Periodic Processing & Transmission ---
        current_time = time.monotonic()
        if current_time - last_process_time >= PROCESS_INTERVAL:
            interval_start_time = last_process_time
            process_and_transmit_results(list(interval_data_R), list(interval_data_L), interval_start_time, ser)
            interval_data_R.clear()
            interval_data_L.clear()
            last_process_time = interval_start_time + PROCESS_INTERVAL

    except serial.SerialException as e: print(f"Serial error: {e}"); break
    except KeyboardInterrupt: print("\nExiting via Ctrl+C."); break
    except Exception as e: print(f"Main loop error: {e}"); traceback.print_exc(); time.sleep(1)

# --- Cleanup ---
print("Cleaning up...")
if ser and ser.is_open: ser.close(); print("Serial port closed.")
