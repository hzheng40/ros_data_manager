#!/usr/bin/env python

from Tkinter import *
from ttk import *

class DataManager(Frame):
    def say_hi(self):
        print 'hi'

    # creating layout for the GUI
    def createWidgets(self):
        # adding quit button
        self.QUIT = Button(self, text='Quit', command=self.quit)
        self.QUIT.pack(side=RIGHT, padx=5, pady=5)
        # example of some sort of function button
        self.hi_there = Button(self, text='Hello', command=self.say_hi)
        self.hi_there.pack(side=RIGHT)
       
        # TODO: add fuction buttons

        # adding a menu
        # TODO: file, edit, view, settings

        # creating menu instance
        self.menu = Menu(self.master)
        self.master.config(menu=self.menu)

        # creating options
        file = Menu(self.menu)
        file.add_command(label='Open...')
        file.add_command(label='Save...')
        edit = Menu(self.menu)
        edit.add_command(label='change some')
        view = Menu(self.menu)
        view.add_command(label='view some')
        settings = Menu(self.menu)
        settings.add_command(label='set some')

        # adding options to menu
        self.menu.add_cascade(label='File', menu=file)
        self.menu.add_cascade(label='Edit', menu=edit)
        self.menu.add_cascade(label='View', menu=view)
        self.menu.add_cascade(label='Settings', menu=settings)

    def __init__(self, master=None):
        #TODO: divide and conquer, with multiple layers of frames
        Frame.__init__(self, master)
        self.master.title('ROS Data Manager')
        self.pack(fill=BOTH, expand=1)
        self.createWidgets()

def main():
    root = Tk()
    root.geometry('1200x800')
    app = DataManager(master=root)
    app.mainloop()
    root.destroy()


if __name__ == '__main__':
    main()