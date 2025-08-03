#ifndef COMMAND_H
#define COMMAND_H

class Command{  //parent Class for commands
    public:
        virtual ~Command(){}
        virtual void run() = 0; //command place holder
};

#endif