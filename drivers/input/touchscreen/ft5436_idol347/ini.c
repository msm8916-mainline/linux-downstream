#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/unistd.h>
#include <linux/slab.h>

#include "ini.h"

#define Section_SpecialSet "SpecialSet"
char CFG_SSL = '[';  /* \CF\EE\B1\EA־\B7\FBSection Symbol --\BFɸ\F9\BE\DD\CC\D8\CA\E2\D0\E8Ҫ\BD\F8\D0ж\A8\D2\E5\B8\FC\B8ģ\AC\C8\E7 { }\B5\C8*/
char CFG_SSR = ']';  /* \CF\EE\B1\EA־\B7\FBSection Symbol --\BFɸ\F9\BE\DD\CC\D8\CA\E2\D0\E8Ҫ\BD\F8\D0ж\A8\D2\E5\B8\FC\B8ģ\AC\C8\E7 { }\B5\C8*/
char CFG_NIS = ':';  /* name \D3\EB index ֮\BC\E4\B5ķָ\F4\B7\FB */
char CFG_NTS = '#';  /* ע\CAͷ\FB*/

char * ini_str_trim_r(char * buf);
char * ini_str_trim_l(char * buf);
static int ini_file_get_line(char *filedata, char *buffer, int maxlen); 
static int ini_split_key_value(char *buf, char **key, char **val); 
static long atol(char *nptr);


/*************************************************************
Function: \BB\F1\B5\C3key\B5\C4ֵ
Input: char * filedata\A1\A1\CEļ\FE\A3\BBchar * section\A1\A1\CF\EEֵ\A3\BBchar * key\A1\A1\BC\FCֵ
Output: char * value\A1\A1key\B5\C4ֵ
Return: 0		SUCCESS
-1		δ\D5ҵ\BDsection
-2		δ\D5ҵ\BDkey
-10		\CEļ\FE\B4\F2\BF\AAʧ\B0\DC
-12		\B6\C1ȡ\CEļ\FEʧ\B0\DC
-14		\CEļ\FE\B8\F1ʽ\B4\ED\CE\F3
-22		\B3\AC\B3\F6\BB\BA\B3\E5\C7\F8\B4\F3С
Note: 
*************************************************************/
int ini_get_key(char *filedata, char * section, char * key, char * value)
{
	/*char buf1[MAX_CFG_BUF + 1];*/
	/*char buf2[MAX_CFG_BUF + 1];*/
	char *buf1 = NULL;
	char *buf2 = NULL;
	
	char *key_ptr, *val_ptr;
	int  n, ret;
	int dataoff = 0;

	buf1 = kmalloc(MAX_CFG_BUF + 1, GFP_KERNEL);
	buf2 = kmalloc(MAX_CFG_BUF + 1, GFP_KERNEL);

	*value='\0';
	//if (strcmp(section, Section_SpecialSet) == 0)
	//	printk("[Focal][%s] search section !");
	while(1) { /* \CB\D1\D5\D2\CF\EEsection */

		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if(n < -1)
			goto r_cfg_end;
		ret = CFG_SECTION_NOT_FOUND;
		if(n < 0)
			goto r_cfg_end; /* \CEļ\FEβ\A3\ACδ\B7\A2\CF\D6 */ 

		if(n > MAX_CFG_BUF)
			goto r_cfg_end;

		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS)
			continue;       /* \BF\D5\D0\D0 \BB\F2 ע\CA\CD\D0\D0 */ 

		ret = CFG_ERR_FILE_FORMAT;
		if(n > 2 && ((buf1[0] == CFG_SSL && buf1[n-1] != CFG_SSR)))
			goto r_cfg_end;
		if(buf1[0] == CFG_SSL) {
			buf1[n-1] = 0x00;
			if(strcmp(buf1+1, section) == 0)
			{
				//if (strcmp(section, Section_SpecialSet) == 0)
				//	printk("[Focal][%s] find section section = %s ! \n", __func__, section);
				break; /* \D5ҵ\BD\CF\EEsection */
			}
		} 
	} 
	//if (strcmp(section, Section_SpecialSet) == 0)
	//	printk("[Focal][%s] search key ! \n", __func__);	
	while(1){ /* \CB\D1\D5\D2key */ 

		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if(n < -1) 
			goto r_cfg_end;
		ret = CFG_KEY_NOT_FOUND;
		if(n < 0)
			goto r_cfg_end;/* \CEļ\FEβ\A3\ACδ\B7\A2\CF\D6key */ 
		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS) 
			continue;       /* \BF\D5\D0\D0 \BB\F2 ע\CA\CD\D0\D0 */ 
		ret = CFG_KEY_NOT_FOUND; 
		if(buf1[0] == CFG_SSL)
		{
			goto r_cfg_end;
		}
		if(buf1[n-1] == '+') { /* \D3\F6+\BAű\EDʾ\CF\C2һ\D0м\CC\D0\F8  */ 		
			buf1[n-1] = 0x00; 
			while(1) {			
				ret = CFG_ERR_READ_FILE; 
				n = ini_file_get_line(filedata+dataoff, buf2, MAX_CFG_BUF);
				dataoff += n;
				if(n < -1) 
					goto r_cfg_end; 
				if(n < 0) 
					break;/* \CEļ\FE\BD\E1\CA\F8 */ 

				n = strlen(ini_str_trim_r(buf2)); 
				ret = CFG_ERR_EXCEED_BUF_SIZE; 
				if(n > 0 && buf2[n-1] == '+'){/* \D3\F6+\BAű\EDʾ\CF\C2һ\D0м\CC\D0\F8 */ 
					buf2[n-1] = 0x00; 
					if( (strlen(buf1) + strlen(buf2)) > MAX_CFG_BUF) 
						goto r_cfg_end; 
					strcat(buf1, buf2); 
					continue; 
				} 
				if(strlen(buf1) + strlen(buf2) > MAX_CFG_BUF) 
					goto r_cfg_end; 
				strcat(buf1, buf2); 
				break; 
			} 
		} 
		ret = CFG_ERR_FILE_FORMAT; 
		if(ini_split_key_value(buf1, &key_ptr, &val_ptr) != 1) 
			goto r_cfg_end; 

		ini_str_trim_l(ini_str_trim_r(key_ptr)); 
		if(strcmp(key_ptr, key) != 0) 
			continue;                                  /* \BA\CDkeyֵ\B2\BBƥ\C5\E4 */ 
		strcpy(value, val_ptr); 
		break; 
	} 
	ret = CFG_OK; 
r_cfg_end:
	//if (strcmp(section, Section_SpecialSet) == 0)
	//	printk("[Focal][%s] at end ret = %d key name = %s key_ptr = %s  ! \n", __func__, ret, key, key_ptr);
	kfree(buf1);
	kfree(buf2);

	return ret; 
} 
/*************************************************************
Function: \BB\F1\B5\C3\CB\F9\D3\D0section
Input:  char *filename\A1\A1\CEļ\FE,int max \D7\EE\B4\F3\BFɷ\B5\BBص\C4section\B5ĸ\F6\CA\FD
Output: char *sections[]\A1\A1\B4\E6\B7\C5section\C3\FB\D7\D6
Return: \B7\B5\BB\D8section\B8\F6\CA\FD\A1\A3\C8\F4\B3\F6\B4\ED\A3\AC\B7\B5\BBظ\BA\CA\FD\A1\A3
-10			\CEļ\FE\B4򿪳\F6\B4\ED
-12			\CEļ\FE\B6\C1ȡ\B4\ED\CE\F3
-14			\CEļ\FE\B8\F1ʽ\B4\ED\CE\F3
Note: 
*************************************************************/
int ini_get_sections(char *filedata, unsigned char * sections[], int max)
{
	//FILE *fp; 
	char buf1[MAX_CFG_BUF + 1]; 
	int n, n_sections = 0, ret; 
	int dataoff = 0;

	while(1) {/*\CB\D1\D5\D2\CF\EEsection */
		ret = CFG_ERR_READ_FILE;
		n = ini_file_get_line(filedata+dataoff, buf1, MAX_CFG_BUF);
		dataoff += n;
		if(n < -1) 
			goto cfg_scts_end; 
		if(n < 0)
			break;/* \CEļ\FEβ */ 
		n = strlen(ini_str_trim_l(ini_str_trim_r(buf1)));
		if(n == 0 || buf1[0] == CFG_NTS) 
			continue;       /* \BF\D5\D0\D0 \BB\F2 ע\CA\CD\D0\D0 */ 
		ret = CFG_ERR_FILE_FORMAT;
		if(n > 2 && ((buf1[0] == CFG_SSL && buf1[n-1] != CFG_SSR)))
			goto cfg_scts_end;
		if(buf1[0] == CFG_SSL) {
			if (max!=0){
				buf1[n-1] = 0x00;
				strcpy((char *)sections[n_sections], buf1+1);
				if (n_sections>=max)
					break;		/* \B3\AC\B9\FD\BFɷ\B5\BB\D8\D7\EE\B4\F3\B8\F6\CA\FD */
			}
			n_sections++;
		} 

	} 
	ret = n_sections;
cfg_scts_end: 
	//	if(fp != NULL)
	//		fclose(fp);
	return ret;
} 


/*************************************************************
Function: ȥ\B3\FD\D7ַ\FB\B4\AE\D3ұߵĿ\D5\D7ַ\FB
Input:  char * buf \D7ַ\FB\B4\AEָ\D5\EB
Output: 
Return: \D7ַ\FB\B4\AEָ\D5\EB
Note: 
*************************************************************/
char * ini_str_trim_r(char * buf)
{
	int len,i;
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);

	memset(tmp,0x00,len);
	for(i = 0;i < len;i++) {
		if (buf[i] !=' ')
			break;
	}
	if (i < len) {
		strncpy(tmp,(buf+i),(len-i));
	}
	strncpy(buf,tmp,len);
	return buf;
}

/*************************************************************
Function: ȥ\B3\FD\D7ַ\FB\B4\AE\D7\F3\B1ߵĿ\D5\D7ַ\FB
Input:  char * buf \D7ַ\FB\B4\AEָ\D5\EB
Output: 
Return: \D7ַ\FB\B4\AEָ\D5\EB
Note: 
*************************************************************/
char * ini_str_trim_l(char * buf)
{
	int len,i;	
	char tmp[512];

	memset(tmp, 0, sizeof(tmp));
	len = strlen(buf);

	memset(tmp,0x00,len);

	for(i = 0;i < len;i++) {
		if (buf[len-i-1] !=' ')
			break;
	}
	if (i < len) {
		strncpy(tmp,buf,len-i);
	}
	strncpy(buf,tmp,len);
	return buf;
}
/*************************************************************
Function: \B4\D3\CEļ\FE\D6ж\C1ȡһ\D0\D0
Input:  FILE *fp \CEļ\FE\BE\E4\B1\FA\A3\BBint maxlen \BB\BA\B3\E5\C7\F8\D7\EE\B4󳤶\C8
Output: char *buffer һ\D0\D0\D7ַ\FB\B4\AE
Return: >0		ʵ\BCʶ\C1\B5ĳ\A4\B6\C8
-1		\CEļ\FE\BD\E1\CA\F8
-2		\B6\C1\CEļ\FE\B3\F6\B4\ED
Note: 
*************************************************************/
static int ini_file_get_line(char *filedata, char *buffer, int maxlen)
{
	int  i, j; 
	char ch1; 

	for (i = 0, j = 0; i < maxlen; j++) { 
		ch1 = filedata[j];
		if (ch1 == '\n' || ch1 == 0x00) 
			break; /* \BB\BB\D0\D0 */ 
		if (ch1 == '\f' || ch1 == 0x1A) {      /* '\f':\BB\BBҳ\B7\FBҲ\CB\E3\D3\D0Ч\D7ַ\FB */ 			
			buffer[i++] = ch1; 
			break; 
		}
		if (ch1 != '\r') 
			buffer[i++] = ch1;    /* \BA\F6\C2Իس\B5\B7\FB */ 
	} 
	buffer[i] = '\0'; 
	return i+2; 
} 
/*************************************************************
Function: \B7\D6\C0\EBkey\BA\CDvalue
key=val
jack   =   liaoyuewang 
|      |   | 
k1     k2  i 
Input:  char *buf
Output: char **key, char **val
Return: 1 --- ok 
0 --- blank line 
-1 --- no key, "= val" 
-2 --- only key, no '=' 
Note: 
*************************************************************/
static int  ini_split_key_value(char *buf, char **key, char **val)
{
	int  i, k1, k2, n; 

	if((n = strlen((char *)buf)) < 1)
		return 0; 
	for(i = 0; i < n; i++) 
		if(buf[i] != ' ' && buf[i] != '\t')
			break; 

	if(i >= n)
		return 0;

	if(buf[i] == '=')
		return -1;

	k1 = i;
	for(i++; i < n; i++) 
		if(buf[i] == '=') 
			break;

	if(i >= n)
		return -2;
	k2 = i;

	for(i++; i < n; i++)
		if(buf[i] != ' ' && buf[i] != '\t') 
			break; 

	buf[k2] = '\0'; 

	*key = buf + k1; 
	*val = buf + i; 
	return 1; 
} 

int my_atoi(const char *str)
{
	int result = 0;
	int signal = 1; /* Ĭ\C8\CFΪ\D5\FD\CA\FD */
	if((*str>='0'&&*str<='9')||*str=='-'||*str=='+') {
		if(*str=='-'||*str=='+') { 
			if(*str=='-')
				signal = -1; /*\CA\E4\C8븺\CA\FD*/
			str++;
		}
	}
	else 
		return 0;
	/*\BF\AAʼת\BB\BB*/
	while(*str>='0' && *str<='9')
		result = result*10 + (*str++ - '0' );

	return signal*result;
}

int isspace(int x)  
{  
	if(x==' '||x=='\t'||x=='\n'||x=='\f'||x=='\b'||x=='\r')  
		return 1;  
	else   
		return 0;  
}  

int isdigit(int x)  
{  
	if(x<='9' && x>='0')           
		return 1;   
	else   
		return 0;  
} 

static long atol(char *nptr)
{
	int c; /* current char */
	long total; /* current total */
	int sign; /* if ''-'', then negative, otherwise positive */
	/* skip whitespace */
	while ( isspace((int)(unsigned char)*nptr) )
		++nptr;
	c = (int)(unsigned char)*nptr++;
	sign = c; /* save sign indication */
	if (c == '-' || c == '+')
		c = (int)(unsigned char)*nptr++; /* skip sign */
	total = 0;
	while (isdigit(c)) {
		total = 10 * total + (c - '0'); /* accumulate digit */
		c = (int)(unsigned char)*nptr++; /* get next char */
	}
	if (sign == '-')
		return -total;
	else
		return total; /* return result, negated if necessary */
}
/***
*int atoi(char *nptr) - Convert string to long
*
*Purpose:
* Converts ASCII string pointed to by nptr to binary.
* Overflow is not detected. Because of this, we can just use
* atol().
*
*Entry:
* nptr = ptr to string to convert
*
*Exit:
* return int value of the string
*
*Exceptions:
* None - overflow is not detected.
*
*******************************************************************************/
int atoi(char *nptr)
{
	return (int)atol(nptr);
}

