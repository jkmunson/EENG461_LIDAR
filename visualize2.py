#!/bin/python3
import pygame
import sys
import math

x1, y1 = 800, 500

def draw_line(screen, color, angle, length):
    global x1, y1
    #x1, y1 = 800, 600
    x2 = int(800 - length * math.sin(math.radians(angle)))
    y2 = int(500 + length * math.cos(math.radians(angle)))
    if not x1 == 0:
        if (math.dist((x1,y1), (x2,y2)) < 250):
            pygame.draw.line(screen, color, (x1, y1), (x2, y2), 2)
        pygame.draw.circle(screen, (255,30,30), (x2, y2), 2)
    x1, y1 = x2, y2

def main():
    # Initialize pygame
    pygame.init()

    # Set up the display
    screen = pygame.display.set_mode((1600, 1000))
    pygame.display.set_caption("Lidar Visualization")

    # Set up the clock to control the frame rate
    clock = pygame.time.Clock()

    # Dictionary to store lines based on angle
    lines = {}
    count = 0
    while True:
        count += 1
        if count > 360:
            lines = {}
            count = 0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        for i in range(36):
            try:
                # Read input in the format [angle] : [length]
                user_input = input()
                angle, length = map(int, user_input.split(':'))
                length = length/4
                if length > 500:
                    length = 500
                # Store the line in the dictionary
                lines[angle] = (angle, length)

            except ValueError:
                a=1

        # Control the frame rate
        # Draw the lines
        screen.fill((20, 20, 20))
        lines = dict(sorted(lines.items(), key=lambda item: item[0]))
        for line in lines.values():
            if line is not None:
                angle, length = line
                draw_line(screen, (30, 230, 0), angle, length)
        x1, y1 = 0, 0
        pygame.draw.circle(screen, (105,105,105), (800,500), 125, width=1);
        pygame.draw.circle(screen, (205,205,205), (800,500), 250, width=1);
        pygame.draw.circle(screen, (105,105,105), (800,500), 375, width=1);
        pygame.draw.circle(screen, (205,205,205), (800,500), 500, width=1);
        pygame.draw.circle(screen, (105,105,105), (800,500), 625, width=1);
        pygame.display.flip()
        #clock.tick(480)


if __name__ == "__main__":
    main()
