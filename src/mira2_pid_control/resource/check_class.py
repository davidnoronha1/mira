from ultralytics import YOLO

# Load model
model = YOLO("/home/adarsh/DNT/mira/src/mira2_pid_control/src/resource/buckets_p1_first_attempt.pt")

# Get class names
names = model.names

print("Classes in model:")
for class_id, class_name in names.items():
    print(f"Class ID {class_id}: {class_name}")