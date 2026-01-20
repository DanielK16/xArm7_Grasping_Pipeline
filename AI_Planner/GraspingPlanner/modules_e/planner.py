import os
import json
import io
import time
import cv2
from PIL import Image
from ollama import chat
from pydantic import BaseModel

from ollama import ListResponse, list
from ollama import ProcessResponse, ps, pull

class JsonResponse(BaseModel):
    check: str
    target_id: int

class OllamaPlanner:
    def __init__(self, cfg):
        print(f">> [AI] Loading Planner: {cfg.OLLAMA_MODEL}")
        self.cfg = cfg
        #self.show_model() #Show all models used
        self.warmup()
        #self.show_usage() # Show model status with CPU/GPU Usage

    def get_action(self, image, objects, facts):
        if not objects: return None

        print(">> [AI] Thinking...")

        # create list for prompt out of json file
        desc = ""
        for o in objects:
            desc += f"- ID {o['id']}: {o['label']} (BBox: {o['bbox_norm']}) | Relation: {o.get('relation')}\n"
        
        # load prompt
        if not os.path.exists(self.cfg.PROMPT_PATH): return {"error": "Prompt file missing"}
        with open(self.cfg.PROMPT_PATH, "r") as f: raw = f.read()
        
        # include list into prompt
        prompt = raw.replace("{object_list}", desc)
        if "{geometric_analysis}" in prompt:
            prompt = prompt.replace("{geometric_analysis}", facts)
        else:
            prompt += f"\n\nGEOMETRIC ANALYSIS:\n{facts}"

        # convert image
        pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        buf = io.BytesIO()
        pil.save(buf, format='JPEG')
        
        #Debug print prompt
        print(prompt)
        
        try:
            res = chat(
                model=self.cfg.OLLAMA_MODEL,
                messages=[{'role':'user', 'content':prompt, 'images':[buf.getvalue()]}],
                format='json',
                options={
                    'temperature': 0,   # controls creativity/randomness
                    'top_k':10,         # limits the models choice to the K most likely next words
                    'top_p': 0.5,       # adds probabilities of words until threshold is reached
                    'num_ctx': 4096,    # size of context space    
                    },
            )
            data = json.loads(res.message.content)
            return data
        except Exception as e:
            print(f"!! [AI] Error: {e}")
            return None


    def warmup(self):
            """
            load ollama model in vram with dummy response!
            """
            print(f">> [AI] Warming up model '{self.cfg.OLLAMA_MODEL}' ")
            try:
                # 1. Create Dummy Image
                dummy_img = Image.new('RGB', (32, 32), color='black')
                buf = io.BytesIO()
                dummy_img.save(buf, format='JPEG')
                
                # 2. Send minimal request
                start = time.perf_counter()
                chat(
                    model=self.cfg.OLLAMA_MODEL,
                    messages=[{
                        'role': 'user', 
                        'content': 'respond with {"status": "ready"}', 
                        'images': [buf.getvalue()]
                    }],
                    format='json'
                )
                duration = time.perf_counter() - start
                print(f">> [AI] Model warmed up and ready! (Took {duration:.2f}s)")
                
            except Exception as e:
                print(f"!! [AI] Warmup failed (is Ollama running?): {e}")

    def show_model(self):
        """
        Show informations about all downloaded models
        """
        response: ListResponse = list()

        for model in response.models:
            print('Name:', model.model)
            print('  Size (MB):', f'{(model.size.real / 1024 / 1024):.2f}')
            if model.details:
                print('  Format:', model.details.format)
                print('  Family:', model.details.family)
                print('  Parameter Size:', model.details.parameter_size)
                print('  Quantization Level:', model.details.quantization_level)
            print('\n')

    def show_usage(self):
        """
        Show GPU/CPU Usage
        """
        response: ProcessResponse = ps()
        for model in response.models:
            print('Model: ', model.model)
            print('  Digest: ', model.digest)
            print('  Expires at: ', model.expires_at)
            print('  Size: ', model.size)
            print('  Size vram: ', model.size_vram)
            print('  Details: ', model.details)
            print('  Context length: ', model.context_length)