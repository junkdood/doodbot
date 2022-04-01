#include "doodbot/player.h"

ChessPlayer::ChessPlayer(){
    _playerstate = ChessPlayer::NOT_INITIALIZED;
}


void ChessPlayer::SetState(int board[3][3]){
    for(int i = 0; i<3;i++){
        for(int j = 0; j<3;j++){
            _board[i][j]= board[i][j];
        }
    }
    _playerstate = ChessPlayer::STATE_SET;
}

void ChessPlayer::PlayChess(){
    //默认有空位置，棋盘已识别
    if(_playerstate == ChessPlayer::NOT_INITIALIZED){
        throw std::runtime_error("state not initialized");
        return;
    }
    do{
        _decisionI = rand()%3;
        _decisionJ = rand()%3;
    }while(_board[_decisionI][_decisionJ]!=Pieces_EMPTY);
    
    _playerstate = ChessPlayer::STATE_SOLVED;
}

int ChessPlayer::GetI(){
    if(_playerstate != ChessPlayer::STATE_SOLVED){
        throw std::runtime_error("state not solved");
        return 0;
    }

    return _decisionI;
}
int ChessPlayer::GetJ(){
    if(_playerstate != ChessPlayer::STATE_SOLVED){
        throw std::runtime_error("state not solved");
        return 0;
    }

    return _decisionJ;
}
int ChessPlayer::Value(){
    return 0;
}

bool ChessPlayer::Win(){
    return false;
}

int ChessPlayer::AlphaBeta(int &value,int deep,bool MAX){
	bool prune=false;
	int i,j,flag,temp;        
    if(deep==3||deep+AlphaBetacount==9){
        return Value();
    }           
    if(Win()==1){
        value=10000;
        return 0;
    }

    if(MAX)                  
        flag=10000;
    else
        flag=-10000;
    for(i=0;i<3 && !prune;i++){
        for(j=0;j<3 && !prune;j++){
            if(AlphaBetaChess[i][j]==0){
                if(MAX){
                    AlphaBetaChess[i][j]=-1;
                    if(Win()==-1)
                        temp=-10000;
                    else
                        temp=AlphaBeta(flag,deep+1,!MAX);
                    if(temp<flag) flag=temp;                      
                    if(flag<=value) prune=true;                      
                }
                else{
                    AlphaBetaChess[i][j]=1;
                    if(Win()==1)
                        temp=10000;
                    else
                        temp=AlphaBeta(flag,deep+1,!MAX);
                    if(temp>flag) flag=temp;                     
                    if(flag>=value) prune=true;
                       
                }
                AlphaBetaChess[i][j]=0;
            }
        }
    }
    if(MAX){
        if(flag>value)
            value=flag;
    }
    else{
        if(flag<value)
            value=flag;
    }
    return flag;
}
