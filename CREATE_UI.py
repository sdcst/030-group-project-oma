import numpy as np
import cv2
import pygame


def user_create_softbody(params):
    pygame.init()

    KEEP_GOING=True
    resolution = params[-1]
    ground_value = params[3]

    X = resolution
    Y = resolution
    
    scrn = pygame.display.set_mode((X, Y))
    pygame.display.set_caption('USER_DRAW_MODE')

    img = np.zeros((resolution, resolution, 3))
    if ground_value!=None:
        GROUND_DISPLAY_COORDS=int(ground_value/32*resolution+resolution/2)
        img[:GROUND_DISPLAY_COORDS] = np.array([102, 102, 102])


    img = np.rot90(img, -1)
    img = np.array(img ,dtype=np.uint8)
    img_surf = pygame.surfarray.make_surface(img)
    node_list=[]
    while KEEP_GOING:


        
        scrn.blit(img_surf, (0, 0))
    
        # paint screen one time
        pygame.display.flip()

        for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    KEEP_GOING = False
                
                if event.type == pygame.MOUSEBUTTONDOWN:

                    MOUSE = pygame.mouse.get_pressed()[0]
                    if MOUSE:
                        POS = pygame.mouse.get_pos()
                        POS = (POS[1], POS[0])
                        img = cv2.circle(img.copy(), POS, int(0.01*resolution), (255, 20 ,20), thickness=-1)
                        img_surf = pygame.surfarray.make_surface(img)
                        node_list.append(POS)
                
                if event.type == pygame.key:
                    pass



if __name__ == "__main__":
    user_create_softbody([0.01, "RK4", -12.0, -6, None,3.4, 100, -0.1, 12, 512])


