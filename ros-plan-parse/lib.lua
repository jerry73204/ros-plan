function pkg_dir(pkg)
   if not string.match(pkg, '[%a%d_-]*') then
      error('"' .. pkg .. '" is not a valid package name')
   end
   local proc = io.popen("ros2 pkg prefix " .. pkg)
   -- local output = proc:read('*all')
   -- proc.close()
   -- return output
end
