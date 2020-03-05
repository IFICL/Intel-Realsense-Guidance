from PIL import Image
import numpy as np 
import torch
from torchvision import transforms

trans = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0], std=[1])
])

a = np.ones([128, 128])
b = Image.fromarray(a)
b = trans(b)
b = b.float()
print(b.shape)
