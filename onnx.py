import torch
import torchvision

# 加载模型
model = torchvision.models.resnet18(pretrained=False)
model.load_state_dict(torch.load('714.pt'))
model.eval()

# 创建一个随机输入张量
input_tensor = torch.randn(1, 3, 224, 224)

# 转换为ONNX
torch.onnx.export(model, input_tensor, 'your_model.onnx', opset_version=11)
