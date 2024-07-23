import gc
import torch

def free_memory():
    # Explicitly invoke garbage collector to free CPU memory
    gc.collect()
    
    # Free GPU memory if CUDA is available
    if torch.cuda.is_available():
        torch.cuda.empty_cache()