#!/usr/bin/python
import web
from multiprocessing.connection import Client

urls = (
  '/send', 'index'
)

class index:
	def POST(self):
		c = Client(('localhost', 5000))
		data = web.input(test="default")
		print data.priority+data.source+data.destination
		c.send((data.priority,data.source,data.destination))
		print('Got:', c.recv())
		return "success"

if __name__ == "__main__":
	app = web.application(urls, globals())
	app.run()