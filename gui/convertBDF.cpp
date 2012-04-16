#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

using namespace std;

string GetSymbol(string str)
{
	string symbols,str_symbols;
	while(str!="")
	{
		symbols=str.substr(0,str.find("\n"));
		str.replace(0,str.find("\n")+1,"");
		str_symbols+="0x"+symbols+",\n";
	}
	return str_symbols;
}



int main(int argc,char* argv[])
{
	
	string str;
	string substr;
	string fontName;
	string fontFile;

	string sname;
	string scode;
	int codes[40000];
	string str_symbols;
	string str_codesymbols;
	int i=0,j=0;
	int prevCode,currCode=0;


	if(argc<3)
	{
		cout<<"wrong parameters"<<endl;
		exit(0);
	}
	else
	{
		fontName=argv[1];
		fontFile=argv[2];
	}
	ifstream	fp(fontFile.c_str(),ios::in);
	int k=0;
	if(fp!=NULL)
	{
		while(!fp.eof())
		{
			str+=(char)fp.get();
			
		}
		fp.close();

		string outPutFileName="Font"+fontName+".c";
		ofstream of(outPutFileName.c_str(),ios::out);

		str.replace(str.rfind("ENDFONT"),str.length()-1,"");

		str_symbols="const char font"+fontName+"[] =\n{\n";

		while(str!="")
		{
			str.replace(0,str.find("STARTCHAR ")+strlen("STARTCHAR "),"");
			
			sname=str.substr(0,str.find("\n"));
			
			str.replace(0,str.find("ENCODING ")+strlen("ENCODING "),"");
			
			scode=str.substr(0,str.find("\n"));
			codes[i]=atoi(scode.c_str());
			str_codesymbols+=scode+",\n";
			

			str.replace(0,str.find("BITMAP")+strlen("BITMAP\n"),"");
			
			substr=str.substr(0,str.find("ENDCHAR"));
			
			str.replace(0,str.find("ENDCHAR")+strlen("ENDCHAR\n"),"");
			
			str_symbols+=GetSymbol(substr)+"\n";

			i++;
		}
		str_codesymbols.replace(str_codesymbols.rfind(","),str_codesymbols.length()-1,"");
		str_symbols.replace(str_symbols.rfind(","),str_symbols.length()-1,"");
		str_codesymbols+="\n};\n";
		str_symbols+="\n};\n";

		of<<"#include \"scrlib.h\"\n\n";
		
		of<<str_symbols<<"\n\n";

		of<<"const struct symbolBlock block"+fontName+"[] =\n{\n";
		j=1;
		for(int m=1;m<i;m++)
		{
			prevCode=codes[m-1];
			currCode=codes[m];
			if(currCode-prevCode==1)
			{
				j++;
			}
			else
			{
				of<<"\t{\n\t\t"<<codes[m-j]<<",\n\t\t"<<j<<",\n\t\t"<<m-1<<"\n\t},"<<endl;
				j=1;
				
			}
		}
		prevCode=codes[i-2];
		currCode=codes[i-1];
		if(currCode-prevCode!=1)
			of<<"\t{\n\t\t"<<codes[i-1]<<",\n\t\t"<<1<<",\n\t\t"<<i-1<<"\n\t}"<<endl;
		of<<"\n};\n";		

		of.close();
	}
	return 0;
}