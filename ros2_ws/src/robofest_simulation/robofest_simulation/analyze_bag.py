"""Validate recorded path waypoints against known mine coordinates."""
from math import hypot
import argparse, json

def verify_paths_clearance(paths, mines, clearance=1.0):
    violations=[]
    for path_index, path in enumerate(paths):
        for waypoint_index, (x,y) in enumerate(path):
            nearest=min((hypot(x-mx,y-my) for mx,my in mines), default=float('inf'))
            if nearest < clearance: violations.append({'path':path_index,'waypoint':waypoint_index,'distance':nearest})
    return violations

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('input', help='JSON export or rosbag2 directory'); parser.add_argument('--clearance',type=float,default=1.0); parser.add_argument('--mines',required=True,help='JSON file containing [[x,y], ...]'); parser.add_argument('--output')
    args=parser.parse_args(argv)
    with open(args.mines,encoding='utf-8') as f: mines=json.load(f)
    with open(args.input,encoding='utf-8') as f: data=json.load(f)
    paths=data.get('paths',data if isinstance(data,list) else [])
    violations=verify_paths_clearance(paths,mines,args.clearance)
    report={'safe':not violations,'clearance_m':args.clearance,'violations':violations,'path_count':len(paths),'mine_count':len(mines)}
    text=json.dumps(report,indent=2)
    if args.output: open(args.output,'w',encoding='utf-8').write(text+'\n')
    else: print(text)
    return 0 if not violations else 2
if __name__=='__main__': raise SystemExit(main())
