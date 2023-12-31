#!/bin/python3
import pygame
import sys
import math

def draw_line(screen, color, angle, length):
    x1, y1 = 800, 600
    x2 = int(x1 + length * math.sin(math.radians(angle)))
    y2 = int(y1 + length * math.cos(math.radians(angle)))
    pygame.draw.line(screen, color, (x1, y1), (x2, y2), 1)

def main():
    # Initialize pygame
    pygame.init()

    # Set up the display
    screen = pygame.display.set_mode((1600, 1200))
    pygame.display.set_caption("Lidar Visualization")

    # Set up the clock to control the frame rate
    clock = pygame.time.Clock()

    # Dictionary to store lines based on angle
    lines = {}

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
        for i in range(36):
            try:
                # Read input in the format [angle] : [length]
                user_input = input()
                angle, length = map(int, user_input.split(':'))
                length = length / 2
                if length > 800:
                    length = 800
                # Store the line in the dictionary
                lines[angle] = (angle, length)

            except ValueError:
                a=1

        # Control the frame rate
        # Draw the lines
        screen.fill((20, 20, 20))
        for line in lines.values():
            if line is not None:
                angle, length = line
                draw_line(screen, (230, 55, 0), angle, length)
        pygame.display.flip()
        #clock.tick(480)


if __name__ == "__main__":
    main()
