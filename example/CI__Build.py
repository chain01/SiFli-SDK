import os
import subprocess
from pathlib import Path

def find_and_build_scons(root_dir='.'):
    """
    递归查找SConstruct文件并执行构建命令（带路径过滤）
    :param root_dir: 起始搜索目录，默认为当前目录
    """
    # 配置参数
    exclude_dirs = {'.git', '.svn', '.idea', '__pycache__', 'venv'}
    exclude_keywords = ['sf32lb56', 'sf32lb58', 'sf32lb52', 'ec-', 'eh-']
    
    try:
        start_path = Path(root_dir).resolve()
        print(f"🔍 开始在目录 {start_path} 中搜索SConstruct文件...")

        found = False

        for root, dirs, files in os.walk(start_path, followlinks=False):
            dirs[:] = [d for d in dirs if d not in exclude_dirs]
            
            if 'SConstruct' in files:
                build_path = Path(root).resolve()
                current_path_str = str(build_path).lower()  # 统一小写匹配

                # 路径过滤检查
                if any(kw in current_path_str for kw in exclude_keywords):
                    print(f"⏭️ 跳过过滤路径 [{build_path}]")
                    continue

                found = True
                print(f"\n🚀 在目录 {build_path} 中找到SConstruct文件")
                print(f"🛠  正在执行: scons --board=em-lb525 -j16")
                
                try:
                    result = subprocess.run(
                        ['scons', '--board=em-lb525', '-j16'],
                        cwd=build_path,
                        check=True,
                        capture_output=True,
                        text=True
                    )
                    print(f"✅ 构建成功完成于 {build_path}")
                except subprocess.CalledProcessError as e:
                    print(f"❌ 构建失败于 {build_path}\n错误信息: {e.stderr}")
                except Exception as e:
                    print(f"⚠️ 发生意外错误于 {build_path}: {str(e)}")

        if not found:
            print("⚠️ 未找到任何可构建的SConstruct文件")

    except KeyboardInterrupt:
        print("\n操作被用户中断")
    except Exception as e:
        print(f"💥 发生全局错误: {str(e)}")

if __name__ == "__main__":
    find_and_build_scons()
