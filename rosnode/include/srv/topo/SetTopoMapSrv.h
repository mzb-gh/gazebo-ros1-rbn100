#ifndef TOPO_MAP_SRV_H
#define TOPO_MAP_SRV_H
#include <string>
#include <vector>

namespace topo_srvs
{
typedef struct _IPOINT
{
    double x;
    double y;
    double theta;
} IPoint;

/**用于站点描述**/
typedef struct __SITE
{
    __SITE(const std::string& _uuid, const IPoint& _pose)
    {
        uuid = _uuid;
        pose = _pose;
    }
    /**
     * @brief uuid uuid描述的站点唯一标识
     */
    std::string uuid;

    /**
     * @brief pose 站点姿态描述
     */
    IPoint pose;
} Site;

/**描述站点间路径**/
typedef struct _ITEM
{
    /**
     * @brief _ITEM 默认构造函数
     */
    _ITEM() {}
    /**
     *描述路径方向的枚举
     */
    enum PATH_PROPERTY
    {
        UNDIRECT,       //节点之间的连接关系为双向,有实际路径连接
        FORWARD,        //有向路径从startIndex到endIndex
        REVERSE,        //有向路径从endIndex到startIndex
        NOPATH,         //这两个节点之间有双向链接,但是没有路径(例如电梯)
        NOPATH_FORWARD, //没有路径,连接关系从startIndex到endIndex(例如电梯)
        NOPATH_REVERSE, //没有路径,连接关系从endIndex到startIndex(例如电梯)
        UNKOWN
    };
    /**
     * @brief _ITEM   构造函数
     * @param _start  起始站点索引
     * @param _end    结束站点索引
     * @param path    储存路径,起始到结束, 注意配合路径从_start站点开始,_end站点结束
     */
    _ITEM(const std::string _start, const std::string _end, PATH_PROPERTY _directFlag, const std::vector<IPoint>& path)
    {
        startIndex = _start;
        endIndex = _end;
        directFlag = _directFlag;
        plannerPlan = path;
    }

    /**
     * @brief _ITEM   构造函数
     * @param _start  起始站点索引
     * @param _end    结束站点索引
     * @param _directFlag  描述路径属性
     * @param path    储存路径,起始到结束
     */
    _ITEM(int _start, int _end, PATH_PROPERTY _directFlag, const std::vector<IPoint>& path)
    {
        startIndex = _start;
        endIndex = _end;
        directFlag = _directFlag;
        plannerPlan = path;
    }

    /**
     * @brief directFlag 记录当前路径描述
     */
    PATH_PROPERTY directFlag = UNDIRECT;
    /**
     * @brief startIndex  起始站点索引uuid
     */
    std::string startIndex;

    /**
     * @brief endIndex    结束站点索引uuid
     */
    std::string endIndex;

    /**
     * @brief plannerPlan 储存路径, 路径是从startIndex到endIndex
     */
    std::vector<IPoint> plannerPlan;
} ITEM;

typedef struct _TOPO_MAP
{
    /**
     * @brief sites  站点列表
     */
    std::vector<Site> sites;

    /**
     * @brief pathList 储存站点间所有链接路径
     */
    std::vector<ITEM> pathList;
} TopoMap;

struct SetTopoMapSrv
{
    struct Request
    {
        TopoMap tMap;
    } request;

    struct Response
    {
        bool success;
    } response;
};
} // namespace topo_srvs
#endif // TOPO_MAP_SRV_H
