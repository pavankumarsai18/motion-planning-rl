import pygame, sys
from pygame.locals import *
import random
import math
import heapq
import copy
import time
from collections import deque


class Person:
    def __init__(self, health, x, y):
        self.health = health
        self.critical = 30
        self.color = (0, 255, 0)
        self.x, self.y = x, y
    
    def get_color(self):
        value = self.health
        red = int((100 - value) / 100 * 255)
        green = int((value / 100) * 255)
        blue = int(value)

        self.color = (red, green, blue)
        return self.color
    
    def isCriticalCondition(self):
        return self.health < self.critical
    
    def __repr__(self):
        '''Returns a string People: x, y, health'''
        return f'Person'

class Robot:
    def __init__(self,x,y):
        self.maxBattery = 100
        self.battery = self.maxBattery
        self.k = 10**-7
        self.time = 0  
        self.savePerson = None
        self.savedPerson = False
        self.color = (0, 255, 0)
        self.x, self.y = x, y
        self.steps = []
        self.numSaved = 0
    
    def get_color(self):
        value = self.battery
        red = int((100 - value) / 100 * 255)
        green = int((value / 100) * 255)
        blue = int(value)

        self.color = (red, green, blue)
        return self.color
    
    def reachedPerson(self):
        return (self.x, self.y) == (self.savePerson.x, self.savePerson.y)

    def hasPersonToSave(self):
        return self.savePerson != None
    
    def assignPerson(self, person):
        self.savePerson = person      

    def tick(self):
        self.battery = self.maxBattery*(math.exp(-self.time*self.k))
        self.time += 0.1

        if self.move():
            self.savedPerson = True
            self.savePerson = None
            return True
        self.savedPerson = False
        return False
    
    def stepsToTake(self, steps):
        self.steps = steps
    
    def move(self):
        if len(self.steps) > 0 and self.battery >= 0.001:
            self.x, self.y = self.steps.popleft()
            # print(f"ROBOT x, y {self.x, self.y}")
            if len(self.steps) == 0:
                self.numSaved += 1
                return True
        return False
    
    def __repr__(self):
        '''Returns a string People: x, y, health'''
        return f'Robot'

class Obstacle:
    def __init__(self, x, y):
        self.x, self.y = x, y
    def __repr__(self):
        '''Returns a string People: x, y, health'''
        return f'Obstacle'


class Grid:
    def __init__(self, n, window_size, screen):
        self.n = n
        self.screen = screen
        self.window_size = window_size
        self.block_size = window_size//n
        

        numBlocks = self.n**2

        self.numRobots = 1
        self.numPeople = numBlocks//20
        self.numObstacles = numBlocks//10

        self.peoplePos = set()
        self.robotPos = set()
        
        self.robots = []
        self.people = []

        self.obstaclePos = set()
        
        self.robotStart = (self.n//2 - 1, self.n//2 )
        
        self.initGrid()

        self.saveQueue = copy.deepcopy(self.people)

        self.numSaved = 0
    
    def printGrid(self):
        ans = []
        for row in self.grid:
            ans_row = []
            for items in row:
                ans_row.append([item.__repr__() for item in items])
            ans.append(row)
        
    def createPeople(self):
        numCritical = self.numPeople//20
        numCreated = 0
        while numCreated < self.numPeople:
            xPos, yPos = random.randint(0, self.n - 1), random.randint(0, self.n - 1)
            while (xPos, yPos) == self.robotStart or len(self.grid[xPos][yPos]) > 0:
                xPos, yPos = random.randint(0, self.n - 1), random.randint(0, self.n - 1)
            
            self.peoplePos.add((xPos, yPos))

            person = Person(health=random.randint(1, 100), x=xPos, y=yPos)
            if numCritical > 0:
                person = Person(health=random.randint(1, 30), x=xPos, y=yPos)
                numCritical -= 1

            self.people.append(person)
            self.grid[xPos][yPos].append(person)  
            numCreated += 1 

    def createRobots(self):
        xPos, yPos = self.robotStart
        numCreated = 0
        while numCreated < self.numRobots:
            robot = Robot(x=xPos, y=yPos)
            self.grid[xPos][yPos].append(robot)
            numCreated += 1
            self.robots.append(robot)
            self.robotPos.add((xPos, yPos))

    def createObstacles(self):
        numCreated = 0
        while numCreated < self.numObstacles:
            xPos, yPos = random.randint(0, self.n - 1), random.randint(0, self.n - 1)
            while (xPos, yPos) == self.robotStart or len(self.grid[xPos][yPos]) > 0:
                xPos, yPos = random.randint(0, self.n - 1), random.randint(0, self.n - 1)
            
            self.obstaclePos.add((xPos, yPos))

            obstacle = Obstacle(xPos, yPos)
            self.grid[xPos][yPos].append(obstacle)  
            numCreated += 1 

    def initGrid(self):
        self.grid = self.makeGrid()
        self.createObstacles()

        self.createRobots()

        self.createPeople()

    def makeGrid(self):
        grid = []
        for rows in range(self.n):
            row = []
            for cols in range(self.n):
                row.append([])
            grid.append(row)
        
        return grid

    def draw(self):
        for x in range(self.n):
            for y in range(self.n + 1):
                if (x, y) == self.robotStart:
                    pygame.draw.rect(self.screen, (0, 0, 255), (x*self.block_size, (y - 1)*self.block_size, self.block_size, self.block_size))
                else:
                    rect = pygame.Rect(x*self.block_size, (y - 1)*self.block_size, self.block_size, self.block_size)
                    pygame.draw.rect(self.screen, (255, 255, 255), rect, 1)

        for x, row in enumerate(self.grid):
            for y, items in enumerate(row):
                for item in items:
                    if type(item) is Person:
                        pygame.draw.circle(
                            self.screen, item.get_color(), (x*self.block_size + self.block_size // 2, (y - 1)*self.block_size + self.block_size // 2),
                            self.block_size // 3
                        )
                    elif type(item) is Obstacle:
                        pygame.draw.rect(
                            self.screen, (255, 0, 0), (x*self.block_size, (y - 1)*self.block_size, self.block_size, self.block_size)
                        )
                    elif type(item) is Robot:
                        points = [(x, y), (x + 0.5, y - 1), (x+1, y)]

                        for i in range(len(points)):
                            x, y = points[i]
                            x, y = x*self.block_size, y*self.block_size
                            points[i] = (x, y)

                        pygame.draw.polygon(self.screen, item.get_color(), points)

    def assignPersonToRobot(self, robot):
        if robot.hasPersonToSave():
            return

        assignedPerson = None
        curDist = float('inf')
        curHealth = 100
        self.saveQueue = sorted(self.people, key=lambda p: (p.health, abs(p.x - robot.x) + abs(p.y - robot.y)))
        
        assignedPerson = self.saveQueue[0]
        if robot and assignedPerson:
            robot.assignPerson(assignedPerson)
            self.saveQueue.remove(assignedPerson)
            
        return


    def dijkstra(self, robot):
        startX, startY = robot.x, robot.y
        start = (startX, startY)
        personToSave = robot.savePerson
        endX, endY = personToSave.x, personToSave.y

        distances = {}
        previous_node = {}

        for r in range(self.n):
            for c in range(self.n):
                distances[(r,c)] = float('inf')
                previous_node[(r, c)] = None
        
        distances[start] = 0
        priority_queue = [(0, start)]  # (distance, node)

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for dr, dc in [(-1, 0), (0, -1), (1, 0), (0, 1)]:
                nextX, nextY = current_node
                nextX += dr
                nextY += dc 

                neighbor = (nextX, nextY)

                if not (0 <= nextX < self.n and 0 <= nextY < self.n):
                    continue
                items = self.grid[nextX][nextY]
                isObstacle = [type(item) is Obstacle for item in items]

                if len(items) > 0 and any(isObstacle):
                    continue
                else:                    
                    distance = current_distance + 1
                    if distance < distances[neighbor]:
                        distances[neighbor] = distance
                        previous_node[neighbor] = current_node
                        heapq.heappush(priority_queue, (distance, neighbor))

        return distances, previous_node

    def shortest_path(self, robot):
        distances, previous_node = self.dijkstra(robot)
        path = []
        start = (robot.x, robot.y)
        end = (robot.savePerson.x, robot.savePerson.y)
        current_node = end
        while current_node is not None:
            path.insert(0, current_node)
            current_node = previous_node[current_node]

        return path if path[0] == start else []
    

    def heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def a_star(self, robot):
        start = (robot.x, robot.y)
        end = (robot.savePerson.x, robot.savePerson.y)

        open_set = {start}
        came_from = {}

        g_score = {}
        f_score = {}

        for r in range(self.n):
            for c in range(self.n):
                g_score[(r,c)] = float('inf')
                f_score[(r, c)] = float('inf')

        g_score[start] = 0
        f_score[start] = self.heuristic(start, end)

        while open_set:
            current = min(open_set, key=lambda pos: f_score[pos])

            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path

            open_set.remove(current)
            for dr, dc in [(-1, 0), (0, -1), (1, 0), (0, 1)]:
                neighbor = (current[0] + dr, current[1] + dc)

                if not (0 <= neighbor[0] < self.n and 0 <= neighbor[1] < self.n):
                    continue
                items = self.grid[neighbor[0]][neighbor[1]]
                is_obstacle = any(isinstance(item, Obstacle) for item in items)

                if is_obstacle:
                    continue

                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, end)
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        return None

    
    def motionPlan(self, robot):
        # a_star_path = self.a_star(robot)
        # if a_star_path:
        #     steps = deque(a_star_path)
        #     path = a_star_path[:-1]
        #     for item in path[::-1]:
        #         steps.append(copy.deepcopy(item))
        #     print(f"A* Path: {a_star_path}")
        #     robot.stepsToTake(steps)
        # else:
        #     print("No path found by A* algorithm")

        path = self.shortest_path(robot)
        steps = deque(copy.deepcopy(path))
        
        path = path[:-1]
        for item in path[::-1]:
            steps.append(copy.deepcopy(item))

        robot.stepsToTake(steps)
            

    def tick(self):

        if not self.saveQueue:
            return 
        for robot in self.robots:

            if not robot.hasPersonToSave():
                self.assignPersonToRobot(robot)
                self.motionPlan(robot)
            
            if robot.reachedPerson():
                self.grid[robot.x][robot.y].remove(robot.savePerson)
                self.people.remove(robot.savePerson)
            

            prevX, prevY = robot.x, robot.y
            
            self.grid[prevX][prevY].remove(robot)
            savedPerson = robot.tick()

            curX, curY = robot.x, robot.y
            self.grid[curX][curY].append(robot)

            if savedPerson:
                robot.savePerson = None
                self.numSaved += 1


def main(): 

    BLACK = (0, 0, 0)
    WINDOW_SIZE = 800

    n = 20

    pygame.init()
    myFont = pygame.font.SysFont('arial', 14)

    SCREEN = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    CLOCK = pygame.time.Clock()

    grid = Grid(n, WINDOW_SIZE, SCREEN)

    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                pygame.quit()
                sys.exit()
        
        
        SCREEN.fill((0,0,0))

        grid.tick()
        grid.draw()

        font = pygame.font.SysFont("comicsans", 18)
        saved = font.render('Saved: ' + str(grid.numSaved), True, (150, 150, 150), (255, 255, 255))
        SCREEN.blit(saved, (WINDOW_SIZE - saved.get_width() - 15, 10))
        print(f"NUM SAVED = {grid.numSaved}")

        pygame.display.update()
        time.sleep(0.1)








if __name__ == "__main__":
    main()




