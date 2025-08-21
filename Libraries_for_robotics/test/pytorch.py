import torch
from PIL import Image
from torchvision.models import resnet18, ResNet18_Weights

img = Image.open("humanoid_robot.jpeg").convert("RGB")

weights = ResNet18_Weights.DEFAULT

model = resnet18(weights=weights).eval()

preprocess = weights.transforms()

with torch.no_grad():
    logits = model(preprocess(img).unsqueeze(0))

probs = logits.softmax(1)[0]


i = int(probs.argmax())

print(weights.meta["categories"][i], float(probs[i]))

