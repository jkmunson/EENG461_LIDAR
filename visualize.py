#!/bin/python3
import turtle

def draw_line(angle, length):
    turtle.setheading(angle)
    turtle.forward(length)

def main():
    # Initialize turtle
    turtle.speed(0)  # Adjust the speed as needed
    turtle.hideturtle()
    turtle.bgcolor("white")

    # Dictionary to store lines based on angle
    lines = {}

    while True:
        try:
            # Read input in the format [angle] : [length]
            user_input = input()
            angle, length = map(int, user_input.split(':'))
            length = length//10
            if length > 500:
                length = 500;
            # Ensure angle is between 0 and 360
            angle %= 360

            # Update or create a new line based on the angle
            if angle in lines:
                # Clear the previous line
                lines[angle].clear()
            else:
                lines[angle] = turtle.Turtle()

            # Move to the center and draw the line
            turtle.penup()
            turtle.goto(0, 0)
            turtle.pendown()
            draw_line(angle, length)

            # Store the line in the dictionary
            lines[angle] = turtle.clone()

        except ValueError:
            print(".")
if __name__ == "__main__":
    main()
    turtle.done()

