#include <cstdio>
#include <queue>
#include <algorithm>
#include <cstring>
#include <string>
#include <ctime>

using std::swap;
using std::priority_queue;
using std::string;

const int MAX_LEN = 9;
const int MAX_DEPTH = 4;
const int DIMENSION = 3;
const int MAX_STATE_NUM = 2000000;
const int dx[] = { 0, -1, 0, 1};
const int dy[] = {-1,  0, 1, 0};

typedef struct _STATE
{
    int item[MAX_LEN];
    int g;
    int fa;
    char fa_direct;

    bool operator == (const _STATE& e) const
    {
        for (int i = 0; i < MAX_LEN; ++i)
        {
            if (item[i] != e.item[i]) return false;
        }
        return true;
    }

    _STATE operator = (const _STATE &e)
    {
        for (int i = 0; i < MAX_LEN; ++i)
        {
            item[i] = e.item[i];
        }
        g = e.g;
        fa = e.fa;
        fa_direct = e.fa_direct;
        return *this;
    }
} STATE, *PSTATE;

typedef struct _NODE
{
    int id;
    int f;

    _NODE() {}
    _NODE(int _id, int _f): id(_id), f(_f) {}

    bool operator < (const _NODE &e) const
    {
        return f > e.f;
    }
} NODE, *PNODE;

STATE start_state;
STATE goal_state;
STATE state_lib[MAX_STATE_NUM];
bool has_path;
string min_path;
time_t start_time;
time_t end_time;

int Read()
{
    int ret = 18;
    for (int i = 0; i < 9; ++i)
    {
        scanf("%d", &start_state.item[i]);
        if (i == 1)
        {
            if (start_state.item[0] == 0 && start_state.item[1] == 0)
            {
                ret = 0;
                return ret;
            }
        }
    }
    for (int i = 0; i < 9; ++i)
    {
        scanf("%d", &goal_state.item[i]);
    }
    return ret;
}

void Init()
{
    min_path = "";
    start_state.g = 0;
    start_state.fa = -1;
    start_state.fa_direct = '\0';
}

int FindZero(const STATE &s)
{
    int ret = 0;
    for (int i = 0; i < MAX_LEN; ++i)
    {
        if (s.item[i] == 0)
        {
            ret = i;
            break;
        }
    }
    return ret;
}

char GetDirect(int x)
{
    char ret = '\0';
    switch (x)
    {
    case 0:
        ret = 'L';
        break;
    case 1:
        ret = 'U';
        break;
    case 2:
        ret = 'R';
        break;
    case 3:
        ret = 'D';
        break;
    default:
        break;
    }
    return ret;
}

string GetPath(int id)
{
    return (state_lib[id].fa == -1) ? "" : (GetPath(state_lib[id].fa) + state_lib[id].fa_direct);
}

int GetH(const STATE &a, const STATE &b)
{
    int ret = 0;
    for (int i = 0; i < MAX_LEN; ++i)
    {
        if (a.item[i] != b.item[i])
        {
            ++ret;
        }
    }
    return ret;
}

int GetY(const STATE &s)
{
    int ret = 0;
    for (int i = 0; i < MAX_LEN; ++i)
    {
        if (s.item[i] == 0) continue;
        for (int j = 0; j < i; ++j)
        {
            if (s.item[j] > s.item[i])
            {
                ++ret;
            }
        }
    }
    return ret;
}

bool Solve()
{
    bool ok = (GetY(start_state) & 1) == (GetY(goal_state) & 1);
    if (ok == false) return ok;
    priority_queue<NODE> open;
    priority_queue<int> closed;
    int state_cnt = 0;
    state_lib[state_cnt] = start_state;
    open.push(NODE(state_cnt, 0 + GetH(state_lib[state_cnt], goal_state)));
    ++state_cnt;
    while (!open.empty())
    {
        int cur_id = open.top().id;
        open.pop();
        STATE &cur_state = state_lib[cur_id];
        closed.push(cur_id);
        if (cur_state == goal_state)
        {
            ok = true;
            min_path = GetPath(cur_id);
            break;
        }

        int location = FindZero(cur_state);
        int x = location / DIMENSION;
        int y = location % DIMENSION;

        for (int i = 0; i < 4; ++i)
        {
            int newx = x + dx[i];
            int newy = y + dy[i];
            if (newx >= 0 && newx < DIMENSION && newy >= 0 && newy <= DIMENSION)
            {
                int new_location = newx * DIMENSION + newy;
                state_lib[state_cnt] = cur_state;
                swap(state_lib[state_cnt].item[location], state_lib[state_cnt].item[new_location]);
                ++state_lib[state_cnt].g;
                state_lib[state_cnt].fa = cur_id;
                state_lib[state_cnt].fa_direct = GetDirect(i);
                open.push(NODE(state_cnt, state_lib[state_cnt].g + GetH(state_lib[state_cnt], goal_state)));
                ++state_cnt;
            }
        }
    }
    return ok;
}

void Output()
{
    if (has_path)
    {
        printf("Path: %s\n", min_path.c_str());
    }
    else
    {
        puts("no solution");
    }
    printf("Use time: %.0fMS\n", (end_time - start_time) * 1000.0 / CLOCKS_PER_SEC);
}

int main()
{
    while (Read() == 18)
    {
        start_time = clock();
        Init();
        has_path = Solve();
        end_time = clock();
        Output();
    }

    return 0;
}

/*

2 8 3 1 0 4 7 6 5

*/
