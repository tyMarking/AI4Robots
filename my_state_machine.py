from __future__ import print_function
import Queue
import threading


class StateMachine():
	diagram = None
	q = None
	state = None
	def __init__(self, diagram, q):
		self.diagram = diagram
		self.q = q

	def threadedRun(self, initState):
		self.state = initState
		while self.q.qsize() > 0:
			event = self.q.get()
			result = self.diagram[self.state][event]
			action = result[0]
			(action())
			state = result[1]


def addPath(diagram, startState, event, func, toState):
	if startState in diagram.keys():
		diagram[startState][event] = [func, toState]
	else:
		diagram[startState] = {event: [func, toState]}


def main():
	diagram = {}
	addPath(diagram, 'B', '1', (lambda : print("FROM B TO A")), 'A')
	addPath(diagram, 'B', '0', (lambda : print("STILL AT B")), 'B')
	addPath(diagram, 'A', '1', (lambda : print("STILL AT A")), 'A')
	addPath(diagram, 'A', '0', (lambda : print("FROM A TO B")), 'B')

	q = Queue.Queue()
	q.put('1')
	q.put('0') #to B
	q.put('0')
	q.put('1') #to A
	FSM = StateMachine(diagram, q)

	fsmThread = threading.Thread(name="FSM Thread", target=FSM.threadedRun, args=['A'])
	fsmThread.daemon = True
	fsmThread.start()
	fsmThread.join()


if __name__ =="__main__":
	main()