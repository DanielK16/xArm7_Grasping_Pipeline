import cv2

def draw_annotations(image, objects):
    img = image.copy()
    for o in objects:
        x1, y1, x2, y2 = o['bbox_pixel']
        label = f"{o['label']} ID:{o['id']}"
        color = o.get('color', (200,200,200))
        
        # Relation anzeigen
        if 'relation' in o and o['relation'] != "ON TABLE":
            cv2.putText(img, o['relation'], (x1, y1+15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

        cv2.rectangle(img, (x1,y1), (x2,y2), color, 2)
        cv2.putText(img, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return img
    
def add_timer(self):
    return None
def rem_timer(self):
    return None
def add_config_lines(self):
    return None
def rem_config_lines(self):
    return None
