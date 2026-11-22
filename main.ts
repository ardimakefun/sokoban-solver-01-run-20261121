import f from"fs";
let W,s,D,U="udlr",F,S,L,p,B,R,i,a,N,v,h,z,y,o;
F=(p,b)=>{for(let k=4;k--;){let q=p+D[k];if(s[q]>"#"&&!R[q]&&!b.includes(q)){R[q]=R[p]+U[k];F(q,b)}}};
S=(b,o,i)=>{let Q=[[b,p,""]],V={};for(h=0;h<Q.length&&h<2.5e5;h++){let[x,y,m]=Q[h];R={[y]:" "};F(y,[...x,...o]);for(a of x)for(let k=4;k--;){let d=D[k],t=a+d;if(R[a-d]&&s[t]>"#"&&!x.includes(t)){N=x.map(z=>z-a?z:t);N.sort();if(!V[a+"|"+N]){v=m+R[a-d]+U[k];if(N.every(z=>/[.*+]/.test(s[z])))return p=a,B[i]=N,v;V[a+"|"+N]=v;Q.push([N,a,v])}}}}return""};
L=t=>{W=t.search("\n")+1;D=[-W,W,-1,1];s=t.replace(/ /g,0);p=t.search("@");B=[[],[],[],[]];t.replace(/[$*]/g,(c,i)=>B[(i%W>7)*2+(i/W>8)].push(i));z="";for(o=4;o--;){if(B[o].length){y=S(B[o],B.flatMap((b,k)=>k-o?b:[]),o);if(!y)return"";z+=y}}return z};
for(i=0;++i<6;){o="/output/i00"+i;f.mkdirSync(o);f.writeFileSync(o+"/solution.txt",L(""+f.readFileSync("/input/i00"+i+"/level.txt")))}
